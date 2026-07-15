-- Hybrid dynamic-yield controller.
--
-- Long safe shared roads use the temporary right-side S manoeuvre. Short roads,
-- obstructed shoulders and candidates that cannot fit the complete train fall
-- back to a temporary one-at-a-time reservation without permanent route marks.

if ADDynamicYield == nil then
    Logging.error("[AD-DY] Hybrid controller could not load")
    return
end

ADDynamicYield.BUILD = "dynamic-yield-exp0.7"
ADDynamicYield.MIN_SHARED_NODES = 2
ADDynamicYield.MIN_SHARED_LENGTH = 5
ADDynamicYield.SERIAL_STOP_DISTANCE = 15
ADDynamicYield.SERIAL_SLOW_DISTANCE = 40
ADDynamicYield.SERIAL_SLOW_SPEED = 15
ADDynamicYield.SERIAL_EXIT_MARGIN = 5
ADDynamicYield.SERIAL_TIMEOUT_MS = 120000

local function dyhDistance(x1, z1, x2, z2)
    return MathUtil.vector2Length(x2 - x1, z2 - z1)
end

local function dyhLabel(vehicle)
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

local function dyhDistanceToIndex(manager, vehicle, route, targetIndex)
    if vehicle == nil or route == nil or targetIndex == nil then
        return math.huge
    end
    local currentRoute, currentIndex = manager:getRoute(vehicle)
    if currentRoute ~= route or currentIndex == nil then
        return math.huge
    end
    if currentIndex > targetIndex then
        return 0
    end

    local x, _, z = manager:getPosition(vehicle)
    local distance = 0
    local previousX = x
    local previousZ = z
    for index = currentIndex, targetIndex do
        local point = route[index]
        if point == nil then
            return math.huge
        end
        distance = distance + dyhDistance(previousX, previousZ, point.x, point.z)
        previousX = point.x
        previousZ = point.z
    end
    return distance
end

local function dyhDistancePastIndex(manager, vehicle, route, exitIndex)
    if vehicle == nil or route == nil or exitIndex == nil then
        return 0
    end
    local currentRoute, currentIndex = manager:getRoute(vehicle)
    if currentRoute ~= route or currentIndex == nil or currentIndex <= exitIndex then
        return 0
    end

    local distance = 0
    for index = exitIndex, currentIndex - 2 do
        local first = route[index]
        local second = route[index + 1]
        if first == nil or second == nil then
            return distance
        end
        distance = distance + dyhDistance(first.x, first.z, second.x, second.z)
    end

    local previous = route[currentIndex - 1]
    if previous ~= nil then
        local x, _, z = manager:getPosition(vehicle)
        distance = distance + dyhDistance(previous.x, previous.z, x, z)
    end
    return distance
end

local function dyhBuildSerialInfo(manager, vehicle, route, currentIndex, entryIndex, exitIndex, approach)
    local speedKmh = (vehicle.lastSpeedReal or 0) * 3600
    local speedMs = speedKmh / 3.6
    local brakingDistance = speedMs * speedMs / 4 + 3
    local grade = manager:getGradeAtRouteDistance(vehicle, route, currentIndex, math.max(0, approach or 0))
    local inside = entryIndex <= currentIndex
    local urgency = math.max(0, 60 - (approach or math.huge))
    local brakingDeficit = math.max(0, brakingDistance + 3 - (approach or math.huge))

    -- Higher score means this vehicle is less desirable to stop. Uphill travel
    -- dominates; speed and insufficient stopping distance refine close cases.
    local priority = grade * 5000 + brakingDeficit * 120 + speedKmh * 4 + urgency * 2
    if inside then
        priority = priority + 10000
    end

    return {
        vehicle = vehicle,
        route = route,
        currentIndex = currentIndex,
        entryIndex = entryIndex,
        exitIndex = exitIndex,
        approach = approach or math.huge,
        grade = grade,
        speedKmh = speedKmh,
        brakingDistance = brakingDistance,
        inside = inside,
        trainLength = manager:getLength(vehicle),
        priority = priority
    }
end

local function dyhChooseSerialThrough(firstInfo, secondInfo)
    if firstInfo.inside ~= secondInfo.inside then
        return firstInfo.inside and firstInfo or secondInfo, "already inside shared road"
    end

    if math.abs(firstInfo.grade - secondInfo.grade) >= 0.02 then
        return firstInfo.grade > secondInfo.grade and firstInfo or secondInfo, "uphill priority"
    end

    if math.abs(firstInfo.priority - secondInfo.priority) >= 1 then
        return firstInfo.priority > secondInfo.priority and firstInfo or secondInfo,
            "stopping difficulty"
    end

    if math.abs(firstInfo.approach - secondInfo.approach) >= 0.5 then
        return firstInfo.approach < secondInfo.approach and firstInfo or secondInfo,
            "closer to entry"
    end

    if math.abs(firstInfo.speedKmh - secondInfo.speedKmh) >= 0.5 then
        return firstInfo.speedKmh > secondInfo.speedKmh and firstInfo or secondInfo,
            "higher current speed"
    end

    -- An exact physical tie is rare. Alternating by reservation number avoids a
    -- permanent directional bias without using a vehicle ID as road priority.
    return (ADDynamicYield.nextPairId % 2 == 1) and firstInfo or secondInfo,
        "equal physical conditions; alternating direction"
end

function ADDynamicYield:createSerialPair(firstVehicle, secondVehicle, overlap, dynamicFailure)
    local firstInfo = dyhBuildSerialInfo(
        self,
        firstVehicle,
        overlap.firstRoute,
        overlap.firstCurrent,
        overlap.firstStart,
        overlap.firstEnd,
        overlap.firstApproachDistance
    )
    local secondInfo = dyhBuildSerialInfo(
        self,
        secondVehicle,
        overlap.secondRoute,
        overlap.secondCurrent,
        overlap.secondStart,
        overlap.secondEnd,
        overlap.secondApproachDistance
    )

    local throughInfo, reason = dyhChooseSerialThrough(firstInfo, secondInfo)
    local yieldInfo = throughInfo == firstInfo and secondInfo or firstInfo
    local pair = {
        id = self.nextPairId,
        mode = "SERIAL",
        phase = "SERIAL_ACTIVE",
        yieldVehicle = yieldInfo.vehicle,
        throughVehicle = throughInfo.vehicle,
        overlap = overlap,
        createdAt = g_time or 0,
        yieldRoute = yieldInfo.route,
        yieldEntryIndex = yieldInfo.entryIndex,
        yieldEntryPoint = yieldInfo.route[yieldInfo.entryIndex],
        throughRoute = throughInfo.route,
        throughExitIndex = throughInfo.exitIndex,
        throughExitPoint = throughInfo.route[throughInfo.exitIndex],
        throughLength = throughInfo.trainLength,
        dynamicFailure = dynamicFailure,
        priorityReason = reason
    }

    self.nextPairId = self.nextPairId + 1
    table.insert(self.pairs, pair)
    self.vehiclePairs[pair.yieldVehicle] = {
        pair = pair,
        role = "yield",
        otherVehicle = pair.throughVehicle
    }
    self.vehiclePairs[pair.throughVehicle] = {
        pair = pair,
        role = "through",
        otherVehicle = pair.yieldVehicle
    }

    self:log(
        "Pair %d hybrid mode=SERIAL: %s continues, %s waits; shared=%.1fm/%d nodes reason=%s dynamic=%s",
        pair.id,
        dyhLabel(pair.throughVehicle),
        dyhLabel(pair.yieldVehicle),
        overlap.sharedLength or -1,
        overlap.nodeCount or -1,
        tostring(reason),
        tostring(dynamicFailure or "not available")
    )
    self:log(
        "Pair %d serial priority: through grade=%.1f%% speed=%.1fkm/h approach=%.1fm; yield grade=%.1f%% speed=%.1fkm/h approach=%.1fm",
        pair.id,
        throughInfo.grade * 100,
        throughInfo.speedKmh,
        throughInfo.approach,
        yieldInfo.grade * 100,
        yieldInfo.speedKmh,
        yieldInfo.approach
    )
    return true
end

local dyhOriginalCreatePair = ADDynamicYield.createPair
function ADDynamicYield:createPair(firstVehicle, secondVehicle, overlap)
    local pairCount = #self.pairs
    local created = dyhOriginalCreatePair(self, firstVehicle, secondVehicle, overlap)
    if created then
        local pair = self.pairs[#self.pairs]
        if pair ~= nil then
            pair.mode = "DYNAMIC"
            self:log("Pair %d hybrid mode=DYNAMIC", pair.id)
        end
        return true
    end

    -- The dynamic builder already checked both shoulders, train length, grades and
    -- available route length. Any failure becomes a serial reservation instead of
    -- falling back to uncoordinated stock traffic behaviour.
    if #self.pairs == pairCount then
        return self:createSerialPair(firstVehicle, secondVehicle, overlap,
            "right-side manoeuvre unavailable or too short")
    end
    return false
end

local dyhOriginalShouldHold = ADDynamicYield.shouldHold
function ADDynamicYield:shouldHold(vehicle)
    local state = self:getPairState(vehicle)
    if state ~= nil and state.pair ~= nil and state.pair.mode == "SERIAL" then
        local pair = state.pair
        if state.role ~= "yield" or pair.phase ~= "SERIAL_ACTIVE" then
            return false
        end

        local distance = dyhDistanceToIndex(self, vehicle, pair.yieldRoute, pair.yieldEntryIndex)
        if distance <= self.SERIAL_STOP_DISTANCE then
            if not pair.serialHoldLogged then
                pair.serialHoldLogged = true
                self:log(
                    "Pair %d serial hold: %s stopped %.1fm before shared entry",
                    pair.id,
                    dyhLabel(vehicle),
                    distance
                )
            end
            return true
        end
        return false
    end
    return dyhOriginalShouldHold(self, vehicle)
end

local dyhOriginalGetDynamicSpeedLimit = ADDynamicYield.getDynamicSpeedLimit
function ADDynamicYield:getDynamicSpeedLimit(vehicle)
    local state = self:getPairState(vehicle)
    if state ~= nil and state.pair ~= nil and state.pair.mode == "SERIAL"
        and state.role == "yield" and state.pair.phase == "SERIAL_ACTIVE" then
        local distance = dyhDistanceToIndex(
            self,
            vehicle,
            state.pair.yieldRoute,
            state.pair.yieldEntryIndex
        )
        if distance <= self.SERIAL_SLOW_DISTANCE then
            return self.SERIAL_SLOW_SPEED
        end
        return nil
    end
    return dyhOriginalGetDynamicSpeedLimit(self, vehicle)
end

function ADDynamicYield:updatePairs()
    for index = #self.pairs, 1, -1 do
        local pair = self.pairs[index]
        local remove = false

        if pair.cleared then
            remove = true
        elseif not self:isActiveVehicle(pair.yieldVehicle)
            or not self:isActiveVehicle(pair.throughVehicle) then
            self:clearPair(pair, "vehicle inactive")
            remove = true
        elseif pair.mode == "SERIAL" then
            if (g_time or 0) - pair.createdAt > self.SERIAL_TIMEOUT_MS then
                self:clearPair(pair, "serial timeout; waiting vehicle released")
                remove = true
            else
                local throughRoute = select(1, self:getRoute(pair.throughVehicle))
                local yieldRoute = select(1, self:getRoute(pair.yieldVehicle))
                if throughRoute ~= pair.throughRoute or yieldRoute ~= pair.yieldRoute then
                    self:clearPair(pair, "serial route changed")
                    remove = true
                else
                    local pastExit = dyhDistancePastIndex(
                        self,
                        pair.throughVehicle,
                        pair.throughRoute,
                        pair.throughExitIndex
                    )
                    local required = (pair.throughLength or 0) + self.SERIAL_EXIT_MARGIN
                    if pastExit >= required then
                        self:log(
                            "Pair %d serial release: %s cleared shared road by %.1fm (required %.1fm)",
                            pair.id,
                            dyhLabel(pair.throughVehicle),
                            pastExit,
                            required
                        )
                        self:clearPair(pair, "serial passage complete")
                        remove = true
                    end
                end
            end
        else
            local candidate = pair.candidate
            if (g_time or 0) - pair.createdAt > self.PAIR_TIMEOUT_MS then
                self:clearPair(pair, "timeout; yielding vehicle released")
                remove = true
            elseif candidate == nil then
                self:clearPair(pair, "dynamic candidate missing")
                remove = true
            else
                local wayPoints, currentIndex = pair.yieldVehicle.ad.drivePathModule:getWayPoints()
                if wayPoints ~= candidate.routeTable or currentIndex == nil then
                    self:clearPair(pair, "yield route changed")
                    remove = true
                elseif pair.phase == "HOLD" and self:throughPassed(pair) then
                    pair.phase = "RELEASED"
                    self:log(
                        "Pair %d releasing %s after %s passed",
                        pair.id,
                        dyhLabel(pair.yieldVehicle),
                        dyhLabel(pair.throughVehicle)
                    )
                elseif pair.phase == "RELEASED" and currentIndex > candidate.generatedEndIndex then
                    self:clearPair(pair, "yield vehicle returned to route")
                    remove = true
                end
            end
        end

        if remove then
            table.remove(self.pairs, index)
        end
    end
end

local dyhOriginalLoadMap = ADDynamicYield.loadMap
function ADDynamicYield:loadMap(name)
    dyhOriginalLoadMap(self, name)
    self.enabled = true
    self.debugEnabled = true
    self:log(
        "%s hybrid controller active; dynamic right yield or serial reservation; shared minimum=%dm/%d nodes",
        self.BUILD,
        self.MIN_SHARED_LENGTH,
        self.MIN_SHARED_NODES
    )
end
