-- Force the temporary yield path onto the actual right-hand side of the vehicle.
-- Route tangent direction can be opposite to the vehicle's physical forward axis
-- on some generated paths, so the perpendicular must be calibrated against the
-- steering node's local +X (right) axis.

if ADDynamicYield == nil then
    Logging.error("[AD-DY] Right-side calibration patch could not load")
    return
end

ADDynamicYield.BUILD = "dynamic-yield-exp0.5"
ADDynamicYield.MAX_PAIR_DISTANCE = 300
ADDynamicYield.MAX_ENTRY_APPROACH_DISTANCE = 100
ADDynamicYield.MIN_PAIR_DISTANCE = 40

local function dyrNormalize(x, z)
    local length = MathUtil.vector2Length(x, z)
    if length < 0.0001 then
        return 1, 0
    end
    return x / length, z / length
end

local function dyrDistance(x1, z1, x2, z2)
    return MathUtil.vector2Length(x2 - x1, z2 - z1)
end

local function dyrGetSteeringNode(vehicle)
    if vehicle ~= nil and vehicle.getAISteeringNode ~= nil then
        local node = vehicle:getAISteeringNode()
        if node ~= nil and entityExists(node) then
            return node
        end
    end
    if vehicle ~= nil and vehicle.components ~= nil and vehicle.components[1] ~= nil then
        return vehicle.components[1].node
    end
    return nil
end

local function dyrLabel(vehicle)
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

local dyrOriginalBuildApproachCandidate = ADDynamicYield.buildApproachCandidate
function ADDynamicYield:buildApproachCandidate(vehicle, otherVehicle, entryDistance, sharedLength)
    local startX, startY, startZ = self:getPosition(vehicle)
    local candidate, reason = dyrOriginalBuildApproachCandidate(
        self,
        vehicle,
        otherVehicle,
        entryDistance,
        sharedLength
    )
    if candidate == nil then
        return nil, reason
    end

    local steeringNode = dyrGetSteeringNode(vehicle)
    if steeringNode == nil or localDirectionToWorld == nil then
        return nil, "cannot determine vehicle right axis"
    end

    local vehicleRightX, _, vehicleRightZ = localDirectionToWorld(steeringNode, 1, 0, 0)
    vehicleRightX, vehicleRightZ = dyrNormalize(vehicleRightX, vehicleRightZ)

    local firstCenter = self:sampleRoute(
        candidate.originalWayPoints,
        candidate.originalCurrentIndex,
        candidate.maneuverStartDistance,
        startX,
        startY,
        startZ
    )
    if firstCenter == nil then
        return nil, "cannot calibrate route side"
    end

    -- Candidate A is (tz, -tx). Flip it when it points against the vehicle's
    -- physical right axis. This makes the decision independent of route node order.
    local baseRightX, baseRightZ = dyrNormalize(firstCenter.tz, -firstCenter.tx)
    local rightDot = baseRightX * vehicleRightX + baseRightZ * vehicleRightZ
    local sideSign = rightDot >= 0 and 1 or -1

    local previousPoint = nil
    for index, point in ipairs(candidate.points or {}) do
        local routeDistance = candidate.maneuverStartDistance + (index - 1) * self.SAMPLE_STEP
        local center = self:sampleRoute(
            candidate.originalWayPoints,
            candidate.originalCurrentIndex,
            routeDistance,
            startX,
            startY,
            startZ
        )
        if center == nil then
            return nil, "right-side route sampling failed"
        end

        local lateralOffset = dyrDistance(center.x, center.z, point.x, point.z)
        local rightX, rightZ = dyrNormalize(center.tz * sideSign, -center.tx * sideSign)
        local pointX = center.x + rightX * lateralOffset
        local pointZ = center.z + rightZ * lateralOffset
        local pointY = self:getTerrainHeight(pointX, pointZ)

        local crossGrade = lateralOffset > 1 and math.abs(pointY - center.y) / lateralOffset or 0
        if crossGrade > self.MAX_CROSS_GRADE then
            return nil, string.format("right-side cross grade %.2f", crossGrade)
        end

        local remapped = {
            x = pointX,
            y = pointY,
            z = pointZ,
            tx = center.tx,
            tz = center.tz,
            routeIndex = center.routeIndex,
            routeT = center.routeT,
            isPathFinderPoint = true
        }

        if previousPoint ~= nil then
            local horizontal = dyrDistance(previousPoint.x, previousPoint.z, remapped.x, remapped.z)
            if horizontal > 0.1 then
                local heightStep = math.abs(remapped.y - previousPoint.y)
                local grade = heightStep / horizontal
                if heightStep > self.MAX_HEIGHT_STEP or grade > self.MAX_LONG_GRADE then
                    return nil, string.format("right-side longitudinal grade %.2f", grade)
                end
            end
        end

        if not self:isCorridorPointClear(vehicle, remapped, candidate.width * 0.5 + 0.6) then
            return nil, "obstacle in actual right corridor"
        end

        candidate.points[index] = remapped
        previousPoint = remapped
    end

    candidate.sideSign = sideSign
    candidate.rightAxisDot = rightDot
    candidate.sideName = "RIGHT"
    return candidate, nil
end

local dyrOriginalCreatePair = ADDynamicYield.createPair
function ADDynamicYield:createPair(firstVehicle, secondVehicle, overlap)
    local created = dyrOriginalCreatePair(self, firstVehicle, secondVehicle, overlap)
    if created then
        local pair = self.pairs[#self.pairs]
        local candidate = pair ~= nil and pair.candidate or nil
        if pair ~= nil and candidate ~= nil then
            self:log(
                "Pair %d physical side confirmed: %s moves RIGHT; routeSign=%d axisDot=%.3f",
                pair.id,
                dyrLabel(candidate.vehicle),
                candidate.sideSign or 0,
                candidate.rightAxisDot or 0
            )
        end
    end
    return created
end

local dyrOriginalLoadMap = ADDynamicYield.loadMap
function ADDynamicYield:loadMap(name)
    dyrOriginalLoadMap(self, name)
    self.enabled = true
    self.debugEnabled = true
    self:log(
        "%s right-side calibration active; pair range=%dm, entry approach=%dm",
        self.BUILD,
        self.MAX_PAIR_DISTANCE,
        self.MAX_ENTRY_APPROACH_DISTANCE
    )
end
