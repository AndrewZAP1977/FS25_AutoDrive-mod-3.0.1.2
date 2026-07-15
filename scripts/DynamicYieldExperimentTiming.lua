-- Synchronize the through vehicle with the yielding train by estimated arrival
-- times. The through vehicle keeps moving whenever its natural ETA to the pass
-- point is later than the train-clear ETA. It is speed-limited only when needed.

if ADDynamicYield == nil then
    Logging.error("[AD-DY] Timing controller could not load")
    return
end

ADDynamicYield.BUILD = "dynamic-yield-exp0.8"
ADDynamicYield.SYNC_TIME_MARGIN = 2.5
ADDynamicYield.SYNC_POINT_MARGIN = 6
ADDynamicYield.SYNC_MIN_ROLL_SPEED = 5
ADDynamicYield.SYNC_MAX_LIMIT = 55
ADDynamicYield.SYNC_EMERGENCY_STOP_DISTANCE = 4
ADDynamicYield.SYNC_LOG_INTERVAL_MS = 2500
ADDynamicYield.SYNC_ACCEL_MARGIN = 1.5

local function dytmClamp(value, minimum, maximum)
    return math.max(minimum, math.min(maximum, value))
end

local function dytmDistance(x1, z1, x2, z2)
    return MathUtil.vector2Length(x2 - x1, z2 - z1)
end

local function dytmLabel(vehicle)
    if vehicle == nil then
        return "none"
    end
    local name = nil
    if vehicle.ad ~= nil and vehicle.ad.stateModule ~= nil and vehicle.ad.stateModule.getName ~= nil then
        name = vehicle.ad.stateModule:getName()
    end
    if (name == nil or name == "") and vehicle.getName ~= nil then
        name = vehicle:getName()
    end
    return string.format("%s#%s", tostring(name or "vehicle"), tostring(vehicle.id or -1))
end

local function dytmGetThroughExitIndex(manager, pair, route)
    if pair == nil or pair.overlap == nil or route == nil then
        return nil
    end
    if route == pair.overlap.firstRoute then
        return pair.overlap.firstEnd
    elseif route == pair.overlap.secondRoute then
        return pair.overlap.secondEnd
    end
    return nil
end

-- Find the remaining route distance from the through vehicle to the longitudinal
-- station of the yielding train's hold point. The hold point itself is offset
-- from the centreline, so each future route segment is projected onto it.
local function dytmDistanceToPassPoint(manager, pair)
    if pair == nil or pair.candidate == nil or pair.candidate.holdPoint == nil then
        return nil, nil
    end

    local vehicle = pair.throughVehicle
    local route, currentIndex = manager:getRoute(vehicle)
    if route == nil or currentIndex == nil then
        return nil, nil
    end

    local exitIndex = dytmGetThroughExitIndex(manager, pair, route)
    if exitIndex == nil then
        return nil, nil
    end
    if currentIndex > exitIndex then
        return 0, 0
    end

    local target = pair.candidate.holdPoint
    local previousX, _, previousZ = manager:getPosition(vehicle)
    local cumulative = 0
    local bestAlong = nil
    local bestCrossSquared = math.huge

    for index = currentIndex, math.min(exitIndex, #route) do
        local point = route[index]
        if point ~= nil then
            local segmentX = point.x - previousX
            local segmentZ = point.z - previousZ
            local segmentLengthSquared = segmentX * segmentX + segmentZ * segmentZ
            if segmentLengthSquared > 0.0001 then
                local t = ((target.x - previousX) * segmentX + (target.z - previousZ) * segmentZ)
                    / segmentLengthSquared
                t = dytmClamp(t, 0, 1)
                local projectedX = previousX + segmentX * t
                local projectedZ = previousZ + segmentZ * t
                local dx = target.x - projectedX
                local dz = target.z - projectedZ
                local crossSquared = dx * dx + dz * dz
                local segmentLength = math.sqrt(segmentLengthSquared)
                if crossSquared < bestCrossSquared then
                    bestCrossSquared = crossSquared
                    bestAlong = cumulative + segmentLength * t
                end
                cumulative = cumulative + segmentLength
            end
            previousX = point.x
            previousZ = point.z
        end
    end

    if bestAlong == nil then
        return nil, nil
    end
    return bestAlong, math.sqrt(bestCrossSquared)
end

local function dytmUpdateObservedYieldSpeed(manager, pair, remainingDistance)
    pair.sync = pair.sync or {}
    local sync = pair.sync
    local now = g_time or 0

    if sync.lastYieldDistance == nil or sync.lastYieldTime == nil then
        sync.lastYieldDistance = remainingDistance
        sync.lastYieldTime = now
        return sync.observedYieldSpeed
    end

    local elapsed = (now - sync.lastYieldTime) / 1000
    if elapsed >= 0.25 then
        local progress = sync.lastYieldDistance - remainingDistance
        if progress >= 0 then
            local measured = progress / elapsed
            if measured >= 0.5 and measured <= 25 then
                if sync.observedYieldSpeed == nil then
                    sync.observedYieldSpeed = measured
                else
                    sync.observedYieldSpeed = sync.observedYieldSpeed * 0.7 + measured * 0.3
                end
            end
        end
        sync.lastYieldDistance = remainingDistance
        sync.lastYieldTime = now
    end

    return sync.observedYieldSpeed
end

function ADDynamicYield:getYieldClearEta(pair)
    if pair == nil or pair.mode ~= "DYNAMIC" or pair.phase ~= "MANEUVER"
        or pair.candidate == nil then
        return 0, 0
    end

    local candidate = pair.candidate
    local remaining = self:distanceToIndex(
        pair.yieldVehicle,
        candidate.routeTable,
        candidate.holdRouteIndex
    )
    if remaining == nil or remaining <= 0 then
        return 0, math.max(remaining or 0, 0)
    end

    local finalDistance = math.min(remaining, self.FINAL_APPROACH_DISTANCE)
    local middleDistance = math.min(
        math.max(remaining - self.FINAL_APPROACH_DISTANCE, 0),
        math.max(self.APPROACH_DISTANCE - self.FINAL_APPROACH_DISTANCE, 0)
    )
    local farDistance = math.max(remaining - self.APPROACH_DISTANCE, 0)

    local currentSpeedKmh = (pair.yieldVehicle.lastSpeedReal or 0) * 3600
    local farSpeedKmh = dytmClamp(currentSpeedKmh > 3 and currentSpeedKmh or self.APPROACH_SPEED,
        8, 45)
    local middleSpeedKmh = dytmClamp(currentSpeedKmh > 3 and currentSpeedKmh or self.APPROACH_SPEED,
        8, self.APPROACH_SPEED)
    local finalSpeedKmh = dytmClamp(currentSpeedKmh > 3 and currentSpeedKmh or self.FINAL_APPROACH_SPEED,
        5, self.FINAL_APPROACH_SPEED)

    local eta = farDistance / (farSpeedKmh / 3.6)
        + middleDistance / (middleSpeedKmh / 3.6)
        + finalDistance / (finalSpeedKmh / 3.6)
        + self.SYNC_ACCEL_MARGIN

    local observedSpeed = dytmUpdateObservedYieldSpeed(self, pair, remaining)
    if observedSpeed ~= nil and observedSpeed > 0.5 then
        eta = math.max(eta, remaining / observedSpeed)
    end

    return eta, remaining
end

function ADDynamicYield:getThroughSynchronization(pair)
    if pair == nil or pair.mode ~= "DYNAMIC" or pair.phase ~= "MANEUVER" then
        return nil
    end

    local throughDistance, crossDistance = dytmDistanceToPassPoint(self, pair)
    if throughDistance == nil then
        return nil
    end

    local clearEta, yieldDistance = self:getYieldClearEta(pair)
    local controlledDistance = math.max(0, throughDistance - self.SYNC_POINT_MARGIN)
    local availableTime = math.max(0.5, clearEta + self.SYNC_TIME_MARGIN)
    local requiredSpeed = controlledDistance / availableTime * 3.6

    pair.sync = pair.sync or {}
    local sync = pair.sync
    local now = g_time or 0
    local previousLimit = sync.speedLimit
    local targetLimit = dytmClamp(requiredSpeed, self.SYNC_MIN_ROLL_SPEED, self.SYNC_MAX_LIMIT)

    if previousLimit == nil then
        sync.speedLimit = targetLimit
    else
        -- Avoid a visibly oscillating speed cap when ETA changes by a few frames.
        sync.speedLimit = previousLimit * 0.75 + targetLimit * 0.25
    end

    sync.throughDistance = throughDistance
    sync.controlledDistance = controlledDistance
    sync.clearEta = clearEta
    sync.yieldDistance = yieldDistance
    sync.requiredSpeed = requiredSpeed
    sync.crossDistance = crossDistance

    if sync.lastLogTime == nil or now - sync.lastLogTime >= self.SYNC_LOG_INTERVAL_MS then
        sync.lastLogTime = now
        self:log(
            "Pair %d timing: yieldRemain=%.1fm clearETA=%.1fs; throughRemain=%.1fm target=%.1fkm/h%s",
            pair.id,
            yieldDistance or -1,
            clearEta,
            throughDistance,
            requiredSpeed,
            requiredSpeed >= self.SYNC_MAX_LIMIT and " (no speed cap)" or ""
        )
    end

    return sync
end

-- Bypass exp0.4's unconditional hold for the dynamic through vehicle. It keeps
-- rolling under ETA control and stops only as a last-resort collision guard.
local dytmOriginalShouldHold = ADDynamicYield.shouldHold
function ADDynamicYield:shouldHold(vehicle)
    local state = self:getPairState(vehicle)
    if state ~= nil and state.pair ~= nil and state.pair.mode == "DYNAMIC"
        and state.role == "through" and state.pair.phase == "MANEUVER" then
        local sync = self:getThroughSynchronization(state.pair)
        if sync == nil then
            return true
        end
        if sync.controlledDistance <= self.SYNC_EMERGENCY_STOP_DISTANCE then
            if not state.pair.syncEmergencyLogged then
                state.pair.syncEmergencyLogged = true
                self:log(
                    "Pair %d timing emergency hold: %s reached pass point before train cleared",
                    state.pair.id,
                    dytmLabel(vehicle)
                )
            end
            return true
        end
        return false
    end
    return dytmOriginalShouldHold(self, vehicle)
end

local dytmOriginalGetDynamicSpeedLimit = ADDynamicYield.getDynamicSpeedLimit
function ADDynamicYield:getDynamicSpeedLimit(vehicle)
    local state = self:getPairState(vehicle)
    if state ~= nil and state.pair ~= nil and state.pair.mode == "DYNAMIC"
        and state.role == "through" and state.pair.phase == "MANEUVER" then
        local sync = self:getThroughSynchronization(state.pair)
        if sync == nil then
            return self.SYNC_MIN_ROLL_SPEED
        end
        if sync.requiredSpeed >= self.SYNC_MAX_LIMIT then
            return nil
        end
        return sync.speedLimit
    end
    return dytmOriginalGetDynamicSpeedLimit(self, vehicle)
end

local dytmOriginalCreatePair = ADDynamicYield.createPair
function ADDynamicYield:createPair(firstVehicle, secondVehicle, overlap)
    local created = dytmOriginalCreatePair(self, firstVehicle, secondVehicle, overlap)
    if created then
        local pair = self.pairs[#self.pairs]
        if pair ~= nil and pair.mode == "DYNAMIC" then
            pair.sync = {}
            local sync = self:getThroughSynchronization(pair)
            if sync ~= nil then
                self:log(
                    "Pair %d timing controller armed: %s keeps moving toward pass point; initial distance=%.1fm target=%.1fkm/h",
                    pair.id,
                    dytmLabel(pair.throughVehicle),
                    sync.throughDistance or -1,
                    sync.requiredSpeed or -1
                )
            end
        end
    end
    return created
end

local dytmOriginalLoadMap = ADDynamicYield.loadMap
function ADDynamicYield:loadMap(name)
    dytmOriginalLoadMap(self, name)
    self.enabled = true
    self.debugEnabled = true
    self:log(
        "%s ETA synchronization active; dynamic through traffic keeps rolling, margin=%.1fs",
        self.BUILD,
        self.SYNC_TIME_MARGIN
    )
end
