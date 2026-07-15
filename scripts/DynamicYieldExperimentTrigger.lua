-- Separate early conflict reservation from visible manoeuvre execution.
-- A pair may be recognized far ahead so stock AutoDrive does not deadlock at the
-- entries, but the yielding train keeps following the centreline until the two
-- vehicles are within a realistic distance along the shared road.

if ADDynamicYield == nil then
    Logging.error("[AD-DY] Manoeuvre-trigger controller could not load")
    return
end

ADDynamicYield.BUILD = "dynamic-yield-exp0.9"
ADDynamicYield.MANEUVER_TRIGGER_ROUTE_DISTANCE = 150
ADDynamicYield.MANEUVER_TRIGGER_WORLD_DISTANCE = 90
ADDynamicYield.MANEUVER_TRIGGER_CHECK_MS = 400
ADDynamicYield.MANEUVER_LATEST_START_MARGIN = 15
ADDynamicYield.ARMED_TIMEOUT_MS = 120000
ADDynamicYield.ARMED_LOG_INTERVAL_MS = 3000

local function dytrDistance(x1, z1, x2, z2)
    return MathUtil.vector2Length(x2 - x1, z2 - z1)
end

local function dytrLabel(vehicle)
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

local function dytrDistanceToIndex(manager, vehicle, route, targetIndex)
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
        distance = distance + dytrDistance(previousX, previousZ, point.x, point.z)
        previousX = point.x
        previousZ = point.z
    end
    return distance
end

local function dytrDistancePastIndex(manager, vehicle, route, entryIndex)
    if vehicle == nil or route == nil or entryIndex == nil then
        return 0
    end
    local currentRoute, currentIndex = manager:getRoute(vehicle)
    if currentRoute ~= route or currentIndex == nil or currentIndex <= entryIndex then
        return 0
    end

    local distance = 0
    for index = entryIndex, currentIndex - 2 do
        local first = route[index]
        local second = route[index + 1]
        if first == nil or second == nil then
            return distance
        end
        distance = distance + dytrDistance(first.x, first.z, second.x, second.z)
    end

    local previous = route[currentIndex - 1]
    if previous ~= nil then
        local x, _, z = manager:getPosition(vehicle)
        distance = distance + dytrDistance(previous.x, previous.z, x, z)
    end
    return distance
end

local function dytrSignedDistanceToEntry(manager, vehicle, route, entryIndex)
    local currentRoute, currentIndex = manager:getRoute(vehicle)
    if currentRoute ~= route or currentIndex == nil then
        return math.huge
    end
    if currentIndex <= entryIndex then
        return dytrDistanceToIndex(manager, vehicle, route, entryIndex)
    end
    return -dytrDistancePastIndex(manager, vehicle, route, entryIndex)
end

local function dytrDistancePastExit(manager, vehicle, route, exitIndex)
    return dytrDistancePastIndex(manager, vehicle, route, exitIndex)
end

local function dytrChooseCandidate(firstCandidate, secondCandidate, firstVehicle, secondVehicle)
    if firstCandidate ~= nil and secondCandidate == nil then
        return firstCandidate, secondVehicle
    elseif secondCandidate ~= nil and firstCandidate == nil then
        return secondCandidate, firstVehicle
    elseif firstCandidate ~= nil and secondCandidate ~= nil then
        if firstCandidate.score <= secondCandidate.score then
            return firstCandidate, secondVehicle
        end
        return secondCandidate, firstVehicle
    end
    return nil, nil
end

local function dytrCurrentOverlap(manager, pair)
    local overlap = pair.overlap
    local firstRoute, firstCurrent = manager:getRoute(pair.firstVehicle)
    local secondRoute, secondCurrent = manager:getRoute(pair.secondVehicle)
    if overlap == nil or firstRoute ~= overlap.firstRoute or secondRoute ~= overlap.secondRoute
        or firstCurrent == nil or secondCurrent == nil then
        return nil
    end

    local firstSigned = dytrSignedDistanceToEntry(
        manager, pair.firstVehicle, firstRoute, overlap.firstStart
    )
    local secondSigned = dytrSignedDistanceToEntry(
        manager, pair.secondVehicle, secondRoute, overlap.secondStart
    )
    if firstSigned == math.huge or secondSigned == math.huge then
        return nil
    end

    local current = {
        firstRoute = firstRoute,
        firstCurrent = firstCurrent,
        firstStart = overlap.firstStart,
        firstEnd = overlap.firstEnd,
        secondRoute = secondRoute,
        secondCurrent = secondCurrent,
        secondStart = overlap.secondStart,
        secondEnd = overlap.secondEnd,
        sharedLength = overlap.sharedLength,
        nodeCount = overlap.nodeCount,
        firstSigned = firstSigned,
        secondSigned = secondSigned,
        firstApproachDistance = math.max(firstSigned, 0),
        secondApproachDistance = math.max(secondSigned, 0),
        firstAvailableShared = math.max(0, overlap.sharedLength + math.min(firstSigned, 0)),
        secondAvailableShared = math.max(0, overlap.sharedLength + math.min(secondSigned, 0))
    }
    current.routeSeparation = math.max(0, overlap.sharedLength + firstSigned + secondSigned)

    local firstX, _, firstZ = manager:getPosition(pair.firstVehicle)
    local secondX, _, secondZ = manager:getPosition(pair.secondVehicle)
    current.worldSeparation = dytrDistance(firstX, firstZ, secondX, secondZ)
    return current
end

local function dytrLogCandidate(manager, pair, candidate, prefix)
    manager:log(
        "Pair %d %s geometry: yield=%s train=%.1fm offset=%.1fm ramp=%.1fm plateau=%.1fm hold=%.1fm total=%.1fm required=%.1fm",
        pair.id,
        prefix,
        dytrLabel(candidate.vehicle),
        candidate.trainLength or candidate.length or 0,
        candidate.offset or 0,
        candidate.ramp or 0,
        candidate.plateau or 0,
        candidate.holdDistance or 0,
        candidate.totalLength or 0,
        candidate.requiredSharedLength or 0
    )
end

-- Final createPair implementation: evaluate the dynamic option now, reserve the
-- two vehicles, but do not inject the temporary S path until the trigger distance.
function ADDynamicYield:createPair(firstVehicle, secondVehicle, overlap)
    local firstCandidate, firstReason = self:buildApproachCandidate(
        firstVehicle,
        secondVehicle,
        overlap.firstApproachDistance or 0,
        overlap.sharedLength
    )
    local secondCandidate, secondReason = self:buildApproachCandidate(
        secondVehicle,
        firstVehicle,
        overlap.secondApproachDistance or 0,
        overlap.sharedLength
    )

    if firstCandidate == nil and secondCandidate == nil then
        self:log(
            "Candidate rejected for %s / %s: %s / %s",
            dytrLabel(firstVehicle),
            dytrLabel(secondVehicle),
            tostring(firstReason),
            tostring(secondReason)
        )
        return self:createSerialPair(
            firstVehicle,
            secondVehicle,
            overlap,
            "right-side manoeuvre unavailable or too short"
        )
    end

    local yieldCandidate, throughVehicle = dytrChooseCandidate(
        firstCandidate,
        secondCandidate,
        firstVehicle,
        secondVehicle
    )
    local pair = {
        id = self.nextPairId,
        mode = "DYNAMIC",
        phase = "ARMED",
        firstVehicle = firstVehicle,
        secondVehicle = secondVehicle,
        yieldVehicle = yieldCandidate.vehicle,
        throughVehicle = throughVehicle,
        candidate = yieldCandidate,
        firstPreviewCandidate = firstCandidate,
        secondPreviewCandidate = secondCandidate,
        firstFailure = firstReason,
        secondFailure = secondReason,
        overlap = overlap,
        createdAt = g_time or 0,
        armedAt = g_time or 0,
        nextTriggerCheck = 0,
        sync = {}
    }

    self.nextPairId = self.nextPairId + 1
    table.insert(self.pairs, pair)
    self.vehiclePairs[firstVehicle] = {
        pair = pair,
        role = firstVehicle == pair.yieldVehicle and "yield" or "through",
        otherVehicle = secondVehicle
    }
    self.vehiclePairs[secondVehicle] = {
        pair = pair,
        role = secondVehicle == pair.yieldVehicle and "yield" or "through",
        otherVehicle = firstVehicle
    }

    local routeSeparation = overlap.sharedLength
        + (overlap.firstApproachDistance or 0)
        + (overlap.secondApproachDistance or 0)
    self:log(
        "Pair %d reserved: %s is planned to yield RIGHT, %s continues; separation=%.1fm, manoeuvre trigger=%dm",
        pair.id,
        dytrLabel(pair.yieldVehicle),
        dytrLabel(pair.throughVehicle),
        routeSeparation,
        self.MANEUVER_TRIGGER_ROUTE_DISTANCE
    )
    dytrLogCandidate(self, pair, yieldCandidate, "preview")
    self:log(
        "Pair %d route reservation: %s approach=%.1fm, %s approach=%.1fm, shared=%.1fm/%d nodes; no pull-off yet",
        pair.id,
        dytrLabel(firstVehicle),
        overlap.firstApproachDistance or -1,
        dytrLabel(secondVehicle),
        overlap.secondApproachDistance or -1,
        overlap.sharedLength or -1,
        overlap.nodeCount or -1
    )
    return true
end

function ADDynamicYield:activateArmedPair(pair, current)
    local firstCandidate, firstReason = self:buildApproachCandidate(
        pair.firstVehicle,
        pair.secondVehicle,
        current.firstApproachDistance,
        current.firstAvailableShared
    )
    local secondCandidate, secondReason = self:buildApproachCandidate(
        pair.secondVehicle,
        pair.firstVehicle,
        current.secondApproachDistance,
        current.secondAvailableShared
    )

    if firstCandidate == nil and secondCandidate == nil then
        self:log(
            "Pair %d cannot start dynamic manoeuvre at trigger: %s / %s; switching to SERIAL",
            pair.id,
            tostring(firstReason),
            tostring(secondReason)
        )
        self:clearPair(pair, "dynamic trigger no longer fits")
        self:createSerialPair(
            pair.firstVehicle,
            pair.secondVehicle,
            current,
            "dynamic manoeuvre no longer fits at trigger"
        )
        return false
    end

    local candidate, throughVehicle = dytrChooseCandidate(
        firstCandidate,
        secondCandidate,
        pair.firstVehicle,
        pair.secondVehicle
    )
    pair.yieldVehicle = candidate.vehicle
    pair.throughVehicle = throughVehicle
    pair.candidate = candidate
    pair.overlap.firstCurrent = current.firstCurrent
    pair.overlap.secondCurrent = current.secondCurrent
    pair.overlap.firstApproachDistance = current.firstApproachDistance
    pair.overlap.secondApproachDistance = current.secondApproachDistance

    self.vehiclePairs[pair.firstVehicle] = {
        pair = pair,
        role = pair.firstVehicle == pair.yieldVehicle and "yield" or "through",
        otherVehicle = pair.secondVehicle
    }
    self.vehiclePairs[pair.secondVehicle] = {
        pair = pair,
        role = pair.secondVehicle == pair.yieldVehicle and "yield" or "through",
        otherVehicle = pair.firstVehicle
    }

    self:injectApproachCandidate(candidate)
    pair.phase = "MANEUVER"
    pair.createdAt = g_time or 0
    pair.activatedAt = g_time or 0
    pair.sync = {}

    self:log(
        "Pair %d manoeuvre triggered: route separation=%.1fm world=%.1fm; %s moves RIGHT, %s keeps approaching",
        pair.id,
        current.routeSeparation,
        current.worldSeparation,
        dytrLabel(pair.yieldVehicle),
        dytrLabel(pair.throughVehicle)
    )
    dytrLogCandidate(self, pair, candidate, "active")
    if candidate.sideName ~= nil then
        self:log(
            "Pair %d physical side confirmed: %s moves %s; routeSign=%d axisDot=%.3f",
            pair.id,
            dytrLabel(candidate.vehicle),
            tostring(candidate.sideName),
            candidate.sideSign or 0,
            candidate.rightAxisDot or 0
        )
    end
    return true
end

-- ARMED pairs intentionally ignore the stock route-conflict stop, but neither
-- vehicle is held or speed-limited by this experiment until the trigger fires.
local dytrOriginalShouldHold = ADDynamicYield.shouldHold
function ADDynamicYield:shouldHold(vehicle)
    local state = self:getPairState(vehicle)
    if state ~= nil and state.pair ~= nil and state.pair.phase == "ARMED" then
        return false
    end
    return dytrOriginalShouldHold(self, vehicle)
end

local dytrOriginalGetDynamicSpeedLimit = ADDynamicYield.getDynamicSpeedLimit
function ADDynamicYield:getDynamicSpeedLimit(vehicle)
    local state = self:getPairState(vehicle)
    if state ~= nil and state.pair ~= nil and state.pair.phase == "ARMED" then
        return nil
    end
    return dytrOriginalGetDynamicSpeedLimit(self, vehicle)
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
                    local pastExit = dytrDistancePastExit(
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
                            dytrLabel(pair.throughVehicle),
                            pastExit,
                            required
                        )
                        self:clearPair(pair, "serial passage complete")
                        remove = true
                    end
                end
            end
        elseif pair.phase == "ARMED" then
            if (g_time or 0) - pair.armedAt > self.ARMED_TIMEOUT_MS then
                self:clearPair(pair, "armed reservation timeout")
                remove = true
            elseif (g_time or 0) >= (pair.nextTriggerCheck or 0) then
                pair.nextTriggerCheck = (g_time or 0) + self.MANEUVER_TRIGGER_CHECK_MS
                local current = dytrCurrentOverlap(self, pair)
                if current == nil then
                    self:clearPair(pair, "armed route changed")
                    remove = true
                else
                    local firstRequired = pair.firstPreviewCandidate ~= nil
                        and pair.firstPreviewCandidate.requiredSharedLength or nil
                    local secondRequired = pair.secondPreviewCandidate ~= nil
                        and pair.secondPreviewCandidate.requiredSharedLength or nil
                    local latestFirst = firstRequired ~= nil
                        and current.firstAvailableShared <= firstRequired + self.MANEUVER_LATEST_START_MARGIN
                    local latestSecond = secondRequired ~= nil
                        and current.secondAvailableShared <= secondRequired + self.MANEUVER_LATEST_START_MARGIN
                    local distanceTrigger = current.routeSeparation <= self.MANEUVER_TRIGGER_ROUTE_DISTANCE
                    local worldTrigger = current.worldSeparation <= self.MANEUVER_TRIGGER_WORLD_DISTANCE
                    local latestSafeTrigger = latestFirst or latestSecond

                    local now = g_time or 0
                    if pair.lastArmedLog == nil or now - pair.lastArmedLog >= self.ARMED_LOG_INTERVAL_MS then
                        pair.lastArmedLog = now
                        self:log(
                            "Pair %d armed: route separation=%.1fm world=%.1fm; trigger=%dm%s",
                            pair.id,
                            current.routeSeparation,
                            current.worldSeparation,
                            self.MANEUVER_TRIGGER_ROUTE_DISTANCE,
                            latestSafeTrigger and " (latest safe start reached)" or ""
                        )
                    end

                    if distanceTrigger or worldTrigger or latestSafeTrigger then
                        local activated = self:activateArmedPair(pair, current)
                        if not activated then
                            remove = true
                        end
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
                        dytrLabel(pair.yieldVehicle),
                        dytrLabel(pair.throughVehicle)
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

local dytrOriginalLoadMap = ADDynamicYield.loadMap
function ADDynamicYield:loadMap(name)
    dytrOriginalLoadMap(self, name)
    self.enabled = true
    self.debugEnabled = true
    self:log(
        "%s delayed manoeuvre active; early reservation remains, pull-off trigger=%dm along shared road",
        self.BUILD,
        self.MANEUVER_TRIGGER_ROUTE_DISTANCE
    )
end
