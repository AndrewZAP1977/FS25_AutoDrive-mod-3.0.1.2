-- Prompt rejoin after a coordinated DUAL pass.
--
-- The longest-train straight section BEFORE the meeting remains untouched so
-- articulated trailers are fully aligned before the opposing vehicle arrives.
-- AFTER the meeting, both vehicles only need to travel half the combined train
-- lengths plus a small longitudinal margin before starting their return ramp.

if ADDynamicYield == nil then
    Logging.error("[AD-DY] Dual-pass exit controller could not load")
    return
end

ADDynamicYield.BUILD = "dynamic-yield-dual-exp0.6"
ADDynamicYield.DUAL_POST_MEETING_MARGIN = 2.0

local function dydeWithPromptExit(manager, firstVehicle, secondVehicle, callback)
    local previousArticulationMargin = manager.DUAL_ARTICULATION_CLEAR_MARGIN
    local previousBaseMargin = manager.DUAL_BASE_CLEAR_MARGIN

    local firstLength = manager:getLength(firstVehicle)
    local secondLength = manager:getLength(secondVehicle)
    local longestLength = math.max(firstLength, secondLength)
    local halfLengthSum = (firstLength + secondLength) * 0.5
    local desiredAfterMeeting = halfLengthSum + manager.DUAL_POST_MEETING_MARGIN

    -- DynamicYieldDualPassTiming calculates:
    --   desiredAfterMeeting = longest + DUAL_ARTICULATION_CLEAR_MARGIN
    --   adaptiveMargin = max(DUAL_BASE_CLEAR_MARGIN,
    --                        desiredAfterMeeting - halfLengthSum)
    -- Feed it values that produce exactly halfLengthSum + the prompt-exit margin,
    -- without duplicating or replacing the already-tested geometry builder.
    manager.DUAL_ARTICULATION_CLEAR_MARGIN = desiredAfterMeeting - longestLength
    manager.DUAL_BASE_CLEAR_MARGIN = manager.DUAL_POST_MEETING_MARGIN

    local result = callback()

    manager.DUAL_ARTICULATION_CLEAR_MARGIN = previousArticulationMargin
    manager.DUAL_BASE_CLEAR_MARGIN = previousBaseMargin
    return result, desiredAfterMeeting, firstLength, secondLength
end

local dydeOriginalCreatePair = ADDynamicYield.createPair
function ADDynamicYield:createPair(firstVehicle, secondVehicle, overlap)
    local created, desiredAfterMeeting, firstLength, secondLength = dydeWithPromptExit(
        self,
        firstVehicle,
        secondVehicle,
        function()
            return dydeOriginalCreatePair(self, firstVehicle, secondVehicle, overlap)
        end
    )

    if created then
        local pair = self.pairs[#self.pairs]
        if pair ~= nil and pair.mode == "DUAL" then
            pair.dualPromptExitDistance = desiredAfterMeeting
            self:log(
                "Pair %d prompt-exit preview: trains=%.1fm/%.1fm, return ramp starts %.1fm after meeting",
                pair.id,
                firstLength,
                secondLength,
                desiredAfterMeeting
            )
        end
    end
    return created
end

local dydeOriginalActivateArmedPair = ADDynamicYield.activateArmedPair
function ADDynamicYield:activateArmedPair(pair, current)
    if pair == nil or pair.mode ~= "DUAL" then
        return dydeOriginalActivateArmedPair(self, pair, current)
    end

    local activated, desiredAfterMeeting, firstLength, secondLength = dydeWithPromptExit(
        self,
        pair.firstVehicle,
        pair.secondVehicle,
        function()
            return dydeOriginalActivateArmedPair(self, pair, current)
        end
    )

    if activated and pair.mode == "DUAL" then
        pair.dualPromptExitDistance = desiredAfterMeeting
        self:log(
            "Pair %d prompt-exit active: trains=%.1fm/%.1fm, both begin rejoin %.1fm after meeting",
            pair.id,
            firstLength,
            secondLength,
            desiredAfterMeeting
        )
    end
    return activated
end

local dydeOriginalLoadMap = ADDynamicYield.loadMap
function ADDynamicYield:loadMap(name)
    dydeOriginalLoadMap(self, name)
    self.enabled = true
    self.debugEnabled = true
    self:log(
        "%s prompt exit active; return begins after half combined train lengths + %.1fm",
        self.BUILD,
        self.DUAL_POST_MEETING_MARGIN
    )
end
