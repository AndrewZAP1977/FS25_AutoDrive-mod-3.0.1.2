-- Conservative limits and approach detection for the dynamic-yield experiment.

if ADDynamicYield == nil then
    Logging.error("[AD-DY] Dynamic-yield safety patch could not load")
    return
end

ADDynamicYield.BUILD = "dynamic-yield-exp0.3"
ADDynamicYield.enabled = true
ADDynamicYield.MIN_SHARED_LENGTH = 120
ADDynamicYield.MAX_ENTRY_APPROACH_DISTANCE = 70

local function dysDistanceToIndex(manager, vehicle, wayPoints, currentIndex, targetIndex)
    if wayPoints == nil or currentIndex == nil or targetIndex == nil or targetIndex < currentIndex then
        return math.huge
    end
    local x, _, z = manager:getPosition(vehicle)
    local distance = 0
    local previousX = x
    local previousZ = z
    for index = currentIndex, targetIndex do
        local point = wayPoints[index]
        if point == nil then
            return math.huge
        end
        distance = distance + MathUtil.vector2Length(point.x - previousX, point.z - previousZ)
        previousX = point.x
        previousZ = point.z
    end
    return distance
end

local dysOriginalFindOpposingOverlap = ADDynamicYield.findOpposingOverlap
function ADDynamicYield:findOpposingOverlap(firstVehicle, secondVehicle)
    local overlap = dysOriginalFindOpposingOverlap(self, firstVehicle, secondVehicle)
    if overlap == nil then
        return nil
    end

    local firstDistance = dysDistanceToIndex(
        self,
        firstVehicle,
        overlap.firstRoute,
        overlap.firstCurrent,
        overlap.firstStart
    )
    local secondDistance = dysDistanceToIndex(
        self,
        secondVehicle,
        overlap.secondRoute,
        overlap.secondCurrent,
        overlap.secondStart
    )

    if firstDistance > self.MAX_ENTRY_APPROACH_DISTANCE
        or secondDistance > self.MAX_ENTRY_APPROACH_DISTANCE then
        local now = g_time or 0
        if self.lastEntryRejectLog == nil or now - self.lastEntryRejectLog > 3000 then
            self.lastEntryRejectLog = now
            self:debug(
                "Shared road entry still too far: %s %.1fm, %s %.1fm; limit=%dm",
                tostring(firstVehicle ~= nil and firstVehicle.id or -1),
                firstDistance,
                tostring(secondVehicle ~= nil and secondVehicle.id or -1),
                secondDistance,
                self.MAX_ENTRY_APPROACH_DISTANCE
            )
        end
        return nil
    end

    overlap.firstApproachDistance = firstDistance
    overlap.secondApproachDistance = secondDistance
    return overlap
end

local dysOriginalLoadMap = ADDynamicYield.loadMap
function ADDynamicYield:loadMap(name)
    dysOriginalLoadMap(self, name)
    self.enabled = true
    self:log(
        "%s active automatically; shared blue road=%dm minimum, entry approach=%dm maximum. Disable with: adDynamicYield off",
        self.BUILD,
        self.MIN_SHARED_LENGTH,
        self.MAX_ENTRY_APPROACH_DISTANCE
    )
end

source(Utils.getFilename("scripts/DynamicYieldExperimentApproach.lua", g_currentModDirectory))
source(Utils.getFilename("scripts/DynamicYieldExperimentTrainSafety.lua", g_currentModDirectory))
source(Utils.getFilename("scripts/DynamicYieldExperimentRightSide.lua", g_currentModDirectory))
source(Utils.getFilename("scripts/DynamicYieldExperimentRouteDetection.lua", g_currentModDirectory))
source(Utils.getFilename("scripts/DynamicYieldExperimentHybrid.lua", g_currentModDirectory))
source(Utils.getFilename("scripts/DynamicYieldExperimentTiming.lua", g_currentModDirectory))
