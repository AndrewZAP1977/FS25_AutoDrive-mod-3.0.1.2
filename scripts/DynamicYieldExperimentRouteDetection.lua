-- Detect an opposing shared road from the future route sequences themselves.
-- Vehicles approaching opposite ends of a long road may be more than 300 m apart
-- and their current headings may be unrelated because of curved entry branches.

if ADDynamicYield == nil then
    Logging.error("[AD-DY] Route-detection patch could not load")
    return
end

ADDynamicYield.BUILD = "dynamic-yield-exp0.6"
ADDynamicYield.MAX_PAIR_DISTANCE = 700
ADDynamicYield.MIN_PAIR_DISTANCE = 0
ADDynamicYield.MAX_ENTRY_APPROACH_DISTANCE = 140
ADDynamicYield.MAX_LOOKAHEAD_NODES = 140
ADDynamicYield.MIN_SHARED_NODES = 3
ADDynamicYield.MIN_SHARED_LENGTH = 35

local function dydDistance(x1, z1, x2, z2)
    return MathUtil.vector2Length(x2 - x1, z2 - z1)
end

local function dydDistanceToIndex(manager, vehicle, wayPoints, currentIndex, targetIndex)
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
        distance = distance + dydDistance(previousX, previousZ, point.x, point.z)
        previousX = point.x
        previousZ = point.z
    end
    return distance
end

local function dydLabel(vehicle)
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

-- Full replacement: matching a sufficiently long sequence in reverse order is
-- itself proof of opposing travel. Current vehicle headings are deliberately not
-- used here.
function ADDynamicYield:findOpposingOverlap(firstVehicle, secondVehicle)
    local firstRoute, firstCurrent = self:getRoute(firstVehicle)
    local secondRoute, secondCurrent = self:getRoute(secondVehicle)
    if firstRoute == nil or secondRoute == nil then
        return nil
    end

    local firstLast = math.min(#firstRoute, firstCurrent + self.MAX_LOOKAHEAD_NODES)
    local secondLast = math.min(#secondRoute, secondCurrent + self.MAX_LOOKAHEAD_NODES)
    local secondIds = {}

    for index = secondCurrent, secondLast do
        local point = secondRoute[index]
        if point ~= nil and point.id ~= nil then
            secondIds[point.id] = secondIds[point.id] or {}
            table.insert(secondIds[point.id], index)
        end
    end

    local best = nil
    for firstStart = firstCurrent, firstLast do
        local point = firstRoute[firstStart]
        local matches = point ~= nil and point.id ~= nil and secondIds[point.id] or nil
        if matches ~= nil then
            for _, secondEnd in ipairs(matches) do
                local count = 0
                local firstIndex = firstStart
                local secondIndex = secondEnd
                while firstIndex <= firstLast and secondIndex >= secondCurrent
                    and firstRoute[firstIndex] ~= nil and secondRoute[secondIndex] ~= nil
                    and firstRoute[firstIndex].id ~= nil
                    and firstRoute[firstIndex].id == secondRoute[secondIndex].id do
                    count = count + 1
                    firstIndex = firstIndex + 1
                    secondIndex = secondIndex - 1
                end

                if count >= self.MIN_SHARED_NODES then
                    local firstEnd = firstStart + count - 1
                    local secondStart = secondEnd - count + 1
                    local sharedLength = self:routeDistance(firstRoute, firstStart, firstEnd)
                    if sharedLength >= self.MIN_SHARED_LENGTH
                        and self:isDualSharedPath(firstRoute, firstStart, firstEnd) then
                        local firstApproach = dydDistanceToIndex(
                            self, firstVehicle, firstRoute, firstCurrent, firstStart
                        )
                        local secondApproach = dydDistanceToIndex(
                            self, secondVehicle, secondRoute, secondCurrent, secondStart
                        )

                        if firstApproach <= self.MAX_ENTRY_APPROACH_DISTANCE
                            and secondApproach <= self.MAX_ENTRY_APPROACH_DISTANCE
                            and (best == nil or sharedLength > best.sharedLength) then
                            best = {
                                firstRoute = firstRoute,
                                firstCurrent = firstCurrent,
                                firstStart = firstStart,
                                firstEnd = firstEnd,
                                secondRoute = secondRoute,
                                secondCurrent = secondCurrent,
                                secondStart = secondStart,
                                secondEnd = secondEnd,
                                firstApproachDistance = firstApproach,
                                secondApproachDistance = secondApproach,
                                sharedLength = sharedLength,
                                nodeCount = count
                            }
                        end
                    end
                end
            end
        end
    end

    return best
end

local dydOriginalCreatePair = ADDynamicYield.createPair
function ADDynamicYield:createPair(firstVehicle, secondVehicle, overlap)
    local created = dydOriginalCreatePair(self, firstVehicle, secondVehicle, overlap)
    if created then
        local pair = self.pairs[#self.pairs]
        self:log(
            "Pair %d route detection: %s approach=%.1fm, %s approach=%.1fm, shared=%.1fm/%d nodes",
            pair ~= nil and pair.id or -1,
            dydLabel(firstVehicle),
            overlap.firstApproachDistance or -1,
            dydLabel(secondVehicle),
            overlap.secondApproachDistance or -1,
            overlap.sharedLength or -1,
            overlap.nodeCount or -1
        )
    end
    return created
end

local dydOriginalLoadMap = ADDynamicYield.loadMap
function ADDynamicYield:loadMap(name)
    dydOriginalLoadMap(self, name)
    self.enabled = true
    self.debugEnabled = true
    self:log(
        "%s route-sequence detection active; range=%dm, entry approach=%dm, current heading ignored",
        self.BUILD,
        self.MAX_PAIR_DISTANCE,
        self.MAX_ENTRY_APPROACH_DISTANCE
    )
end
