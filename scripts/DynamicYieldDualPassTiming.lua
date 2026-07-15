-- Refine DUAL timing and post-meeting train clearance after the main dual-pass
-- controller loads.
--
-- Timing uses one absolute arrival schedule. It does not feed a speed limit back
-- into the next ETA estimate, which previously drove both vehicles toward 5 km/h.
-- The generated parallel section is also extended adaptively so the longest train
-- is fully straight and clear before either vehicle begins returning to centre.

if ADDynamicYield == nil then
    Logging.error("[AD-DY] Dual-pass timing refinement could not load")
    return
end

ADDynamicYield.BUILD = "dynamic-yield-dual-exp0.3"
ADDynamicYield.DUAL_MIN_ROLL_SPEED = 10
ADDynamicYield.DUAL_SYNC_START_APPROACH = 25
ADDynamicYield.DUAL_SYNC_TIME_MARGIN = 1.0
ADDynamicYield.DUAL_ARTICULATION_CLEAR_MARGIN = 6
ADDynamicYield.DUAL_BASE_CLEAR_MARGIN = ADDynamicYield.DUAL_CLEAR_MARGIN or 4

local function dydtClamp(value, minimum, maximum)
    return math.max(minimum, math.min(maximum, value))
end

local function dydtDistanceToIndex(manager, vehicle, candidate, targetIndex)
    if vehicle == nil or candidate == nil or candidate.routeTable == nil
        or targetIndex == nil then
        return nil
    end
    local distance = manager:distanceToIndex(vehicle, candidate.routeTable, targetIndex)
    if distance == nil or distance == math.huge then
        return nil
    end
    return distance
end

local function dydtDistanceToMeeting(manager, vehicle, candidate)
    return dydtDistanceToIndex(manager, vehicle, candidate, candidate.meetingRouteIndex)
end

local function dydtDistanceToManeuverStart(manager, vehicle, candidate)
    return dydtDistanceToIndex(manager, vehicle, candidate, candidate.generatedStartIndex)
end

local function dydtAdaptiveClearMargin(manager, firstVehicle, secondVehicle)
    local firstLength = manager:getLength(firstVehicle)
    local secondLength = manager:getLength(secondVehicle)
    local longestLength = math.max(firstLength, secondLength)
    local desiredAfterMeeting = longestLength + manager.DUAL_ARTICULATION_CLEAR_MARGIN
    local halfLengthSum = (firstLength + secondLength) * 0.5
    local adaptiveMargin = math.max(
        manager.DUAL_BASE_CLEAR_MARGIN,
        desiredAfterMeeting - halfLengthSum
    )
    return adaptiveMargin, desiredAfterMeeting, firstLength, secondLength
end

local function dydtWithAdaptiveClearance(manager, firstVehicle, secondVehicle, callback)
    local previousMargin = manager.DUAL_CLEAR_MARGIN
    local adaptiveMargin, desiredAfterMeeting, firstLength, secondLength =
        dydtAdaptiveClearMargin(manager, firstVehicle, secondVehicle)
    manager.DUAL_CLEAR_MARGIN = adaptiveMargin
    local result = callback()
    manager.DUAL_CLEAR_MARGIN = previousMargin
    return result, adaptiveMargin, desiredAfterMeeting, firstLength, secondLength
end

-- The main DUAL planner calculates:
--   clearAfterMeeting = (firstLength + secondLength) / 2 + DUAL_CLEAR_MARGIN
-- Temporarily replacing that margin lets the existing tested geometry builder
-- produce max(train lengths) + articulation margin without duplicating the planner.
local dydtOriginalCreatePair = ADDynamicYield.createPair
function ADDynamicYield:createPair(firstVehicle, secondVehicle, overlap)
    local created, adaptiveMargin, desiredAfterMeeting, firstLength, secondLength =
        dydtWithAdaptiveClearance(self, firstVehicle, secondVehicle, function()
            return dydtOriginalCreatePair(self, firstVehicle, secondVehicle, overlap)
        end)

    if created then
        local pair = self.pairs[#self.pairs]
        if pair ~= nil and pair.mode == "DUAL" then
            pair.dualAdaptiveClearMargin = adaptiveMargin
            pair.dualDesiredAfterMeeting = desiredAfterMeeting
            self:log(
                "Pair %d dual train clearance preview: trains=%.1fm/%.1fm, straight after meeting=%.1fm, adaptive margin=%.1fm",
                pair.id,
                firstLength,
                secondLength,
                desiredAfterMeeting,
                adaptiveMargin
            )
        end
    end
    return created
end

local dydtOriginalActivateArmedPair = ADDynamicYield.activateArmedPair
function ADDynamicYield:activateArmedPair(pair, current)
    if pair == nil or pair.mode ~= "DUAL" then
        return dydtOriginalActivateArmedPair(self, pair, current)
    end

    local activated, adaptiveMargin, desiredAfterMeeting, firstLength, secondLength =
        dydtWithAdaptiveClearance(self, pair.firstVehicle, pair.secondVehicle, function()
            return dydtOriginalActivateArmedPair(self, pair, current)
        end)

    if activated and pair.mode == "DUAL" then
        pair.dualAdaptiveClearMargin = adaptiveMargin
        pair.dualDesiredAfterMeeting = desiredAfterMeeting
        pair.dualSync = {}
        self:log(
            "Pair %d dual train clearance active: trains=%.1fm/%.1fm, both remain parallel %.1fm after meeting",
            pair.id,
            firstLength,
            secondLength,
            desiredAfterMeeting
        )
    end
    return activated
end

local function dydtGetDistances(manager, pair)
    if pair == nil or pair.firstCandidate == nil or pair.secondCandidate == nil then
        return nil
    end

    local firstMeeting = dydtDistanceToMeeting(
        manager,
        pair.firstVehicle,
        pair.firstCandidate
    )
    local secondMeeting = dydtDistanceToMeeting(
        manager,
        pair.secondVehicle,
        pair.secondCandidate
    )
    local firstStart = dydtDistanceToManeuverStart(
        manager,
        pair.firstVehicle,
        pair.firstCandidate
    )
    local secondStart = dydtDistanceToManeuverStart(
        manager,
        pair.secondVehicle,
        pair.secondCandidate
    )

    if firstMeeting == nil or secondMeeting == nil
        or firstStart == nil or secondStart == nil then
        return nil
    end

    return firstMeeting, secondMeeting, firstStart, secondStart
end

local function dydtInitializeArrivalSchedule(manager, pair, now, firstDistance, secondDistance)
    pair.dualSync = pair.dualSync or {}
    local baseSpeedMs = manager.DUAL_PASS_SPEED / 3.6
    local duration = math.max(firstDistance, secondDistance) / math.max(baseSpeedMs, 0.5)
        + manager.DUAL_SYNC_TIME_MARGIN
    pair.dualSync.arrivalTime = now + duration * 1000
    pair.dualSync.firstLimit = nil
    pair.dualSync.secondLimit = nil
    manager:log(
        "Pair %d dual arrival schedule armed: %.1fs to meeting at up to %dkm/h",
        pair.id,
        duration,
        manager.DUAL_PASS_SPEED
    )
end

local function dydtGetLimit(manager, pair, vehicle)
    if pair.phase == "DUAL_CLEARING" then
        return manager.DUAL_CLEAR_SPEED
    end

    local firstDistance, secondDistance, firstStart, secondStart = dydtGetDistances(manager, pair)
    if firstDistance == nil then
        return manager.DUAL_MIN_ROLL_SPEED
    end

    -- Before either vehicle gets near its lateral path both retain their normal AD
    -- road speed. This avoids slowing them immediately at the 150 m pair trigger.
    if math.min(firstStart, secondStart) > manager.DUAL_SYNC_START_APPROACH then
        return nil
    end

    local now = g_time or 0
    pair.dualSync = pair.dualSync or {}
    if pair.dualSync.arrivalTime == nil then
        dydtInitializeArrivalSchedule(manager, pair, now, firstDistance, secondDistance)
    end

    local baseSpeedMs = manager.DUAL_PASS_SPEED / 3.6
    local remainingTime = math.max(
        (pair.dualSync.arrivalTime - now) / 1000,
        0.5
    )

    -- If acceleration, a turn or a heavy train made the original schedule
    -- impossible, move the common arrival time later. Never shorten it from an
    -- already imposed speed cap: that is what prevents the old 5 km/h feedback.
    local minimumFeasibleTime = math.max(firstDistance, secondDistance)
        / math.max(baseSpeedMs, 0.5)
    if minimumFeasibleTime > remainingTime then
        remainingTime = minimumFeasibleTime + manager.DUAL_SYNC_TIME_MARGIN
        pair.dualSync.arrivalTime = now + remainingTime * 1000
    end

    local firstTarget = dydtClamp(
        firstDistance / remainingTime * 3.6,
        manager.DUAL_MIN_ROLL_SPEED,
        manager.DUAL_PASS_SPEED
    )
    local secondTarget = dydtClamp(
        secondDistance / remainingTime * 3.6,
        manager.DUAL_MIN_ROLL_SPEED,
        manager.DUAL_PASS_SPEED
    )

    local function smooth(previous, target)
        if previous == nil then
            return target
        end
        local weight = target > previous and 0.50 or 0.20
        return previous * (1 - weight) + target * weight
    end

    pair.dualSync.firstLimit = smooth(pair.dualSync.firstLimit, firstTarget)
    pair.dualSync.secondLimit = smooth(pair.dualSync.secondLimit, secondTarget)

    if pair.dualSync.lastStableLog == nil
        or now - pair.dualSync.lastStableLog >= manager.DUAL_LOG_INTERVAL_MS then
        pair.dualSync.lastStableLog = now
        manager:log(
            "Pair %d stable dual timing: %s %.1fm start=%.1fm @ %.1fkm/h; %s %.1fm start=%.1fm @ %.1fkm/h; ETA %.1fs",
            pair.id,
            tostring(pair.firstVehicle.id or -1),
            firstDistance,
            firstStart,
            pair.dualSync.firstLimit,
            tostring(pair.secondVehicle.id or -1),
            secondDistance,
            secondStart,
            pair.dualSync.secondLimit,
            remainingTime
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
        "%s stable ETA active; normal speed retained until %dm before lateral path, rolling minimum=%dkm/h, articulation margin=%dm",
        self.BUILD,
        self.DUAL_SYNC_START_APPROACH,
        self.DUAL_MIN_ROLL_SPEED,
        self.DUAL_ARTICULATION_CLEAR_MARGIN
    )
end
