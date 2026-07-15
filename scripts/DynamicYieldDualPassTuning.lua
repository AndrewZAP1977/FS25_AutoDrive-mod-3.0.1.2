-- Final user-facing tuning for coordinated DUAL passing.
--
-- Pair reservation and route construction may still happen early for safety, but
-- they remain invisible to the driver. Speed control starts only near each
-- vehicle's own lateral S-curve. The full-train straight geometry from exp0.4 is
-- intentionally unchanged.

if ADDynamicYield == nil then
    Logging.error("[AD-DY] Dual-pass user tuning could not load")
    return
end

ADDynamicYield.BUILD = "dynamic-yield-dual-exp0.5"
ADDynamicYield.DUAL_PASS_SPEED = 25
ADDynamicYield.DUAL_CLEAR_SPEED = 25
ADDynamicYield.DUAL_MIN_ROLL_SPEED = 15
ADDynamicYield.DUAL_SYNC_START_APPROACH = 25
ADDynamicYield.DUAL_SYNC_TIME_MARGIN = 0.25
ADDynamicYield.DUAL_APPROACH_DECEL_MS2 = 2.50
ADDynamicYield.DUAL_LIMIT_DECEL_KMH_PER_SEC = 10.0
ADDynamicYield.DUAL_LIMIT_ACCEL_KMH_PER_SEC = 10.0
ADDynamicYield.DUAL_VISIBLE_SPEED_CONTROL_DISTANCE = 25

local function dydvCandidateForVehicle(pair, vehicle)
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

local function dydvDistanceToStart(manager, vehicle, candidate)
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

-- The exp0.4 refinement exposes a comfortable braking envelope as soon as the
-- temporary route is injected. Suppress that cap until the vehicle is within the
-- requested 25 m of its own lateral path. This changes only speed control; route
-- geometry, obstacle checks and train-clearance distances remain untouched.
local dydvOriginalGetDynamicSpeedLimit = ADDynamicYield.getDynamicSpeedLimit
function ADDynamicYield:getDynamicSpeedLimit(vehicle)
    local state = self:getPairState(vehicle)
    if state ~= nil and state.pair ~= nil and state.pair.mode == "DUAL"
        and state.pair.phase == "DUAL_ACTIVE" then
        local candidate = dydvCandidateForVehicle(state.pair, vehicle)
        local distanceToStart = dydvDistanceToStart(self, vehicle, candidate)
        if distanceToStart ~= nil
            and distanceToStart > self.DUAL_VISIBLE_SPEED_CONTROL_DISTANCE then
            return nil
        end
    end
    return dydvOriginalGetDynamicSpeedLimit(self, vehicle)
end

local dydvOriginalLoadMap = ADDynamicYield.loadMap
function ADDynamicYield:loadMap(name)
    dydvOriginalLoadMap(self, name)
    self.enabled = true
    self.debugEnabled = true
    self:log(
        "%s tuning active; normal AD speed until %dm before lateral path, DUAL pass/clear speed=%dkm/h",
        self.BUILD,
        self.DUAL_VISIBLE_SPEED_CONTROL_DISTANCE,
        self.DUAL_PASS_SPEED
    )
end
