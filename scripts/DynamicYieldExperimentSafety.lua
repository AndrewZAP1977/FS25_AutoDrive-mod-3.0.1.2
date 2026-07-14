-- Additional conservative limits for the first dynamic-yield experiment.

if ADDynamicYield == nil then
    Logging.error("[AD-DY] Dynamic-yield safety patch could not load")
    return
end

-- The temporary S path for ordinary vehicles is roughly 65-100 m long. Require
-- a clearly longer common two-way road so the whole manoeuvre stays on it.
ADDynamicYield.MIN_SHARED_LENGTH = 120
ADDynamicYield.MAX_OVERLAP_START_NODES = 2

local dySafetyOriginalFindOpposingOverlap = ADDynamicYield.findOpposingOverlap
function ADDynamicYield:findOpposingOverlap(firstVehicle, secondVehicle)
    local overlap = dySafetyOriginalFindOpposingOverlap(self, firstVehicle, secondVehicle)
    if overlap == nil then
        return nil
    end

    if overlap.firstStart > overlap.firstCurrent + self.MAX_OVERLAP_START_NODES
        or overlap.secondStart > overlap.secondCurrent + self.MAX_OVERLAP_START_NODES then
        self:debug(
            "Opposing overlap rejected because shared road starts later: %s +%d nodes, %s +%d nodes",
            tostring(firstVehicle ~= nil and firstVehicle.id or -1),
            overlap.firstStart - overlap.firstCurrent,
            tostring(secondVehicle ~= nil and secondVehicle.id or -1),
            overlap.secondStart - overlap.secondCurrent
        )
        return nil
    end

    return overlap
end

local dySafetyOriginalLoadMap = ADDynamicYield.loadMap
function ADDynamicYield:loadMap(name)
    dySafetyOriginalLoadMap(self, name)
    self:log(
        "Safety limits: current shared blue road only, minimum shared length=%dm",
        self.MIN_SHARED_LENGTH
    )
end
