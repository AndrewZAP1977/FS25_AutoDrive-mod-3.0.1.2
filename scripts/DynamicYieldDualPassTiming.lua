-- Refine DUAL ETA synchronization after the main dual-pass controller loads.
-- Near-zero speed immediately after activation or a route turn is not treated as
-- the train's sustainable speed; otherwise both vehicles could be capped at the
-- minimum rolling speed for most of the approach.

if ADDynamicYield == nil then
    Logging.error("[AD-DY] Dual-pass timing refinement could not load")
    return
end

ADDynamicYield.BUILD = "dynamic-yield-dual-exp0.2"
ADDynamicYield.DUAL_OBSERVED_SPEED_THRESHOLD = 5

local function dydtClamp(value, minimum, maximum)
    return math.max(minimum, math.min(maximum, value))
end

local function dydtDistanceToMeeting(manager, vehicle, candidate)
    if vehicle == nil or candidate == nil or candidate.routeTable == nil
        or candidate.meetingRouteIndex == nil then
        return nil
    end
    local distance = manager:distanceToIndex(
        vehicle,
        candidate.routeTable,
        candidate.meetingRouteIndex
    )
    if distance == nil or distance == math.huge then
        return nil
    end
    return distance
end

local function dydtObservedSpeedMs(manager, vehicle)
    local speedKmh = (vehicle.lastSpeedReal or 0) * 3600
    if speedKmh < manager.DUAL_OBSERVED_SPEED_THRESHOLD then
        return manager.DUAL_PASS_SPEED / 3.6
    end
    return math.min(speedKmh, manager.DUAL_PASS_SPEED) / 3.6
end

local function dydtGetLimit(manager, pair, vehicle)
    if pair.phase == "DUAL_CLEARING" then
        return manager.DUAL_CLEAR_SPEED
    end

    local firstDistance = dydtDistanceToMeeting(
        manager,
        pair.firstVehicle,
        pair.firstCandidate
    )
    local secondDistance = dydtDistanceToMeeting(
        manager,
        pair.secondVehicle,
        pair.secondCandidate
    )
    if firstDistance == nil or secondDistance == nil then
        return manager.DUAL_MIN_ROLL_SPEED
    end

    local baseSpeedMs = manager.DUAL_PASS_SPEED / 3.6
    local firstSpeedMs = dydtObservedSpeedMs(manager, pair.firstVehicle)
    local secondSpeedMs = dydtObservedSpeedMs(manager, pair.secondVehicle)
    local targetTime = math.max(
        firstDistance / math.max(firstSpeedMs, 0.5),
        secondDistance / math.max(secondSpeedMs, 0.5),
        math.max(firstDistance, secondDistance) / math.max(baseSpeedMs, 0.5),
        0.5
    )

    local firstTarget = dydtClamp(
        firstDistance / targetTime * 3.6,
        manager.DUAL_MIN_ROLL_SPEED,
        manager.DUAL_PASS_SPEED
    )
    local secondTarget = dydtClamp(
        secondDistance / targetTime * 3.6,
        manager.DUAL_MIN_ROLL_SPEED,
        manager.DUAL_PASS_SPEED
    )

    pair.dualSync = pair.dualSync or {}
    local firstPrevious = pair.dualSync.firstLimit
    local secondPrevious = pair.dualSync.secondLimit
    pair.dualSync.firstLimit = firstPrevious == nil
        and firstTarget or firstPrevious * 0.75 + firstTarget * 0.25
    pair.dualSync.secondLimit = secondPrevious == nil
        and secondTarget or secondPrevious * 0.75 + secondTarget * 0.25

    local now = g_time or 0
    if pair.dualSync.lastRefinedLog == nil
        or now - pair.dualSync.lastRefinedLog >= manager.DUAL_LOG_INTERVAL_MS then
        pair.dualSync.lastRefinedLog = now
        manager:log(
            "Pair %d refined dual timing: %s %.1fm @ %.1fkm/h; %s %.1fm @ %.1fkm/h",
            pair.id,
            tostring(pair.firstVehicle.id or -1),
            firstDistance,
            pair.dualSync.firstLimit,
            tostring(pair.secondVehicle.id or -1),
            secondDistance,
            pair.dualSync.secondLimit
        )
    end

    return vehicle == pair.firstVehicle
        and pair.dualSync.firstLimit or pair.dualSync.secondLimit
end

local dydtOriginalGetDynamicSpeedLimit = ADDynamicYield.getDynamicSpeedLimit
function ADDynamicYield:getDynamicSpeedLimit(vehicle)
    local state = self:getPairState(vehicle)
    if state ~= nil and state.pair ~= nil and state.pair.mode == "DUAL" then
        if state.pair.phase == "ARMED" then
            return nil
        end
        if state.pair.phase == "DUAL_ACTIVE" or state.pair.phase == "DUAL_CLEARING" then
            return dydtGetLimit(self, state.pair, vehicle)
        end
        return nil
    end
    return dydtOriginalGetDynamicSpeedLimit(self, vehicle)
end

local dydtOriginalLoadMap = ADDynamicYield.loadMap
function ADDynamicYield:loadMap(name)
    dydtOriginalLoadMap(self, name)
    self.enabled = true
    self.debugEnabled = true
    self:log(
        "%s refined ETA active; observed speed below %dkm/h ignored during dual startup",
        self.BUILD,
        self.DUAL_OBSERVED_SPEED_THRESHOLD
    )
end
