-- Final refinement for coordinated two-vehicle passing.
--
-- 1. Reserve the DUAL pair earlier so speed reduction can be progressive while
--    the visible lateral manoeuvre still begins at a realistic closing distance.
-- 2. Keep both vehicles fully parallel for the longest train length plus margin
--    BEFORE the meeting point as well as after it. This lets articulated trailers
--    finish following the lateral ramp before the opposing vehicle reaches them.
-- 3. Apply a distance-based braking envelope and a time-based limit slew rate so
--    the speed cap cannot jump directly from normal road speed to ~15 km/h.

if ADDynamicYield == nil then
    Logging.error("[AD-DY] Dual-pass final refinement could not load")
    return
end

ADDynamicYield.BUILD = "dynamic-yield-dual-exp0.4"
ADDynamicYield.DUAL_TRIGGER_DISTANCE = 220
ADDynamicYield.DUAL_SYNC_START_APPROACH = 55
ADDynamicYield.DUAL_PRE_ARTICULATION_MARGIN = 6
ADDynamicYield.DUAL_APPROACH_DECEL_MS2 = 1.15
ADDynamicYield.DUAL_LIMIT_DECEL_KMH_PER_SEC = 4.0
ADDynamicYield.DUAL_LIMIT_ACCEL_KMH_PER_SEC = 6.0
ADDynamicYield.DUAL_LIMIT_STATE_TIMEOUT_MS = 1500

local function dydrClamp(value, minimum, maximum)
    return math.max(minimum, math.min(maximum, value))
end

local function dydrLabel(vehicle)
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

local function dydrLongestTrain(manager, firstVehicle, secondVehicle)
    local firstLength = manager:getLength(firstVehicle)
    local secondLength = manager:getLength(secondVehicle)
    return math.max(firstLength, secondLength), firstLength, secondLength
end

local function dydrWithPreMeetingClearance(manager, firstVehicle, secondVehicle, callback)
    local previous = manager.DUAL_PRE_MEETING_MARGIN
    local longest, firstLength, secondLength = dydrLongestTrain(manager, firstVehicle, secondVehicle)
    local required = longest + manager.DUAL_PRE_ARTICULATION_MARGIN
    manager.DUAL_PRE_MEETING_MARGIN = math.max(previous or 0, required)
    local result = callback()
    manager.DUAL_PRE_MEETING_MARGIN = previous
    return result, required, firstLength, secondLength
end

local dydrOriginalCreatePair = ADDynamicYield.createPair
function ADDynamicYield:createPair(firstVehicle, secondVehicle, overlap)
    local created, required, firstLength, secondLength = dydrWithPreMeetingClearance(
        self,
        firstVehicle,
        secondVehicle,
        function()
            return dydrOriginalCreatePair(self, firstVehicle, secondVehicle, overlap)
        end
    )

    if created then
        local pair = self.pairs[#self.pairs]
        if pair ~= nil and pair.mode == "DUAL" then
            pair.dualRequiredBeforeMeeting = required
            self:log(
                "Pair %d dual articulation preview: trains=%.1fm/%.1fm, both fully parallel %.1fm before meeting",
                pair.id,
                firstLength,
                secondLength,
                required
            )
        end
    end
    return created
end

local dydrOriginalActivateArmedPair = ADDynamicYield.activateArmedPair
function ADDynamicYield:activateArmedPair(pair, current)
    if pair == nil or pair.mode ~= "DUAL" then
        return dydrOriginalActivateArmedPair(self, pair, current)
    end

    local activated, required, firstLength, secondLength = dydrWithPreMeetingClearance(
        self,
        pair.firstVehicle,
        pair.secondVehicle,
        function()
            return dydrOriginalActivateArmedPair(self, pair, current)
        end
    )

    if activated and pair.mode == "DUAL" then
        pair.dualRequiredBeforeMeeting = required
        pair.dualRateLimit = setmetatable({}, {__mode = "k"})
        self:log(
            "Pair %d dual articulation active: trains=%.1fm/%.1fm, both remain parallel %.1fm before meeting",
            pair.id,
            firstLength,
            secondLength,
            required
        )
    end
    return activated
end

local function dydrDistanceToStart(manager, vehicle, candidate)
    if candidate == nil or candidate.routeTable == nil or candidate.generatedStartIndex == nil then
        return nil
    end
    local distance = manager:distanceToIndex(
        vehicle,
        candidate.routeTable,
        candidate.generatedStartIndex
    )
    if distance == nil or distance == math.huge then
        return nil
    end
    return math.max(distance, 0)
end

local function dydrCandidateForVehicle(pair, vehicle)
    if pair == nil then
        return nil
    end
    if vehicle == pair.firstVehicle then
        return pair.firstCandidate
    elseif vehicle == pair.secondVehicle then
        return pair.secondCandidate
    end
    return nil
end

-- Maximum safe speed at the current distance from the lateral path assuming a
-- comfortable constant deceleration to DUAL_PASS_SPEED at the path start.
local function dydrApproachEnvelope(manager, distanceToStart)
    local entrySpeedMs = manager.DUAL_PASS_SPEED / 3.6
    local speedMs = math.sqrt(
        entrySpeedMs * entrySpeedMs
            + 2 * manager.DUAL_APPROACH_DECEL_MS2 * math.max(distanceToStart or 0, 0)
    )
    return speedMs * 3.6
end

local function dydrRateLimitedCap(manager, pair, vehicle, target, distanceToStart)
    pair.dualRateLimit = pair.dualRateLimit or setmetatable({}, {__mode = "k"})
    local state = pair.dualRateLimit[vehicle]
    local now = g_time or 0
    local actualSpeed = math.max((vehicle.lastSpeedReal or 0) * 3600, 0)
    local envelope = dydrApproachEnvelope(manager, distanceToStart)

    if state == nil or now - (state.lastTime or now) > manager.DUAL_LIMIT_STATE_TIMEOUT_MS then
        state = {
            value = math.max(target, math.min(actualSpeed > 1 and actualSpeed or envelope, envelope)),
            lastTime = now
        }
        pair.dualRateLimit[vehicle] = state
    end

    local elapsed = math.max((now - state.lastTime) / 1000, 0)
    state.lastTime = now

    local desired = math.min(target, envelope)
    if distanceToStart <= 0 then
        desired = math.min(desired, manager.DUAL_PASS_SPEED)
    end

    if desired < state.value then
        state.value = math.max(
            desired,
            state.value - manager.DUAL_LIMIT_DECEL_KMH_PER_SEC * elapsed
        )
    else
        state.value = math.min(
            desired,
            state.value + manager.DUAL_LIMIT_ACCEL_KMH_PER_SEC * elapsed
        )
    end

    -- The physical braking envelope is a hard ceiling. The slew limiter controls
    -- comfort, while this ceiling guarantees no excessive speed at the S-curve.
    state.value = math.min(state.value, envelope)
    if distanceToStart <= 0 then
        state.value = math.min(state.value, manager.DUAL_PASS_SPEED)
    end
    return state.value
end

local dydrOriginalGetDynamicSpeedLimit = ADDynamicYield.getDynamicSpeedLimit
function ADDynamicYield:getDynamicSpeedLimit(vehicle)
    local state = self:getPairState(vehicle)
    if state == nil or state.pair == nil or state.pair.mode ~= "DUAL" then
        return dydrOriginalGetDynamicSpeedLimit(self, vehicle)
    end

    local pair = state.pair
    if pair.phase == "ARMED" then
        return nil
    end

    if pair.phase ~= "DUAL_ACTIVE" and pair.phase ~= "DUAL_CLEARING" then
        return dydrOriginalGetDynamicSpeedLimit(self, vehicle)
    end

    local candidate = dydrCandidateForVehicle(pair, vehicle)
    local distanceToStart = dydrDistanceToStart(self, vehicle, candidate)
    if distanceToStart == nil then
        return dydrOriginalGetDynamicSpeedLimit(self, vehicle)
    end

    local synchronizedTarget = dydrOriginalGetDynamicSpeedLimit(self, vehicle)
    if pair.phase == "DUAL_CLEARING" then
        synchronizedTarget = synchronizedTarget or self.DUAL_CLEAR_SPEED
    elseif synchronizedTarget == nil then
        -- Well before synchronization begins, only the progressive braking envelope
        -- applies. If it is above the current AD limit it has no visible effect.
        synchronizedTarget = dydrApproachEnvelope(self, distanceToStart)
    end

    return dydrRateLimitedCap(
        self,
        pair,
        vehicle,
        synchronizedTarget,
        distanceToStart
    )
end

local dydrOriginalLoadMap = ADDynamicYield.loadMap
function ADDynamicYield:loadMap(name)
    dydrOriginalLoadMap(self, name)
    self.enabled = true
    self.debugEnabled = true
    self:log(
        "%s active; DUAL trigger=%dm, progressive deceleration %.2fm/s2, full-train straight margin=%dm before and after meeting",
        self.BUILD,
        self.DUAL_TRIGGER_DISTANCE,
        self.DUAL_APPROACH_DECEL_MS2,
        self.DUAL_PRE_ARTICULATION_MARGIN
    )
end
