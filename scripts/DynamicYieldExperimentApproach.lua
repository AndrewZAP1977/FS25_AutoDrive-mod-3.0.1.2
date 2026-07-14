-- Detect opposing traffic before the shared road entry, then begin the temporary
-- right-side manoeuvre only after the yielding vehicle has entered that road.

if ADDynamicYield == nil then
    Logging.error("[AD-DY] Approach-planning patch could not load")
    return
end

ADDynamicYield.BUILD = "dynamic-yield-exp0.3"
ADDynamicYield.MAX_PAIR_DISTANCE = 220
ADDynamicYield.ENTRY_BUFFER = 12
ADDynamicYield.REJOIN_MARGIN = 10
ADDynamicYield.debugEnabled = true

local function dyaClamp(value, minimum, maximum)
    return math.max(minimum, math.min(maximum, value))
end

local function dyaDistance(x1, z1, x2, z2)
    return MathUtil.vector2Length(x2 - x1, z2 - z1)
end

local function dyaSmoothStep(value)
    value = dyaClamp(value, 0, 1)
    return value * value * (3 - 2 * value)
end

local function dyaLabel(vehicle)
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

function ADDynamicYield:getGradeAtRouteDistance(vehicle, wayPoints, currentIndex, startDistance)
    local x, y, z = self:getPosition(vehicle)
    local first = self:sampleRoute(wayPoints, currentIndex, startDistance, x, y, z)
    local second = self:sampleRoute(wayPoints, currentIndex, startDistance + 25, x, y, z)
    if first == nil or second == nil then
        return 0
    end
    return (second.y - first.y) / math.max(dyaDistance(first.x, first.z, second.x, second.z), 1)
end

function ADDynamicYield:buildApproachCandidate(vehicle, otherVehicle, entryDistance, sharedLength)
    local wayPoints, currentIndex = self:getRoute(vehicle)
    if wayPoints == nil then
        return nil, "no route"
    end

    local x, y, z = self:getPosition(vehicle)
    local width = self:getWidth(vehicle)
    local otherWidth = self:getWidth(otherVehicle)
    local length = self:getLength(vehicle)
    local otherLength = self:getLength(otherVehicle)
    local turnRadius = self:getTurnRadius(vehicle)
    local offset = dyaClamp(width * 0.5 + otherWidth * 0.5 + self.CLEARANCE_MARGIN,
        self.MIN_OFFSET, self.MAX_OFFSET)
    local ramp = dyaClamp(math.max(self.MIN_RAMP, turnRadius * 2.2, length * 0.32),
        self.MIN_RAMP, self.MAX_RAMP)
    local plateau = dyaClamp(self.MIN_PLATEAU + length * 0.18,
        self.MIN_PLATEAU, self.MAX_PLATEAU)
    local totalLength = ramp * 2 + plateau
    local holdDistance = ramp + plateau * 0.5
    local maneuverStartDistance = math.max(0, entryDistance) + self.ENTRY_BUFFER

    if self.ENTRY_BUFFER + totalLength + self.REJOIN_MARGIN > sharedLength then
        return nil, string.format("shared road too short after entry (need %.1fm, have %.1fm)",
            self.ENTRY_BUFFER + totalLength + self.REJOIN_MARGIN, sharedLength)
    end

    if self:routeDistance(wayPoints, currentIndex, #wayPoints) < maneuverStartDistance + totalLength + 5 then
        return nil, "insufficient future route"
    end

    local points = {}
    local holdPointIndex = nil
    local previousPoint = nil
    local maxLongGrade = 0
    local maxCrossGrade = 0
    local localDistance = 0

    while localDistance <= totalLength + 0.001 do
        local center = self:sampleRoute(
            wayPoints,
            currentIndex,
            maneuverStartDistance + localDistance,
            x,
            y,
            z
        )
        if center == nil then
            return nil, "route sampling failed"
        end

        local profile
        if localDistance < ramp then
            profile = dyaSmoothStep(localDistance / ramp)
        elseif localDistance <= ramp + plateau then
            profile = 1
        else
            profile = 1 - dyaSmoothStep((localDistance - ramp - plateau) / ramp)
        end

        local appliedOffset = offset * profile
        local pointX = center.x + center.tz * appliedOffset
        local pointZ = center.z - center.tx * appliedOffset
        local pointY = self:getTerrainHeight(pointX, pointZ)
        local crossGrade = appliedOffset > 1 and math.abs(pointY - center.y) / appliedOffset or 0
        maxCrossGrade = math.max(maxCrossGrade, crossGrade)
        if crossGrade > self.MAX_CROSS_GRADE then
            return nil, string.format("cross grade %.2f", crossGrade)
        end

        local temporary = {
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
            local horizontal = dyaDistance(previousPoint.x, previousPoint.z, temporary.x, temporary.z)
            if horizontal > 0.1 then
                local heightStep = math.abs(temporary.y - previousPoint.y)
                local grade = heightStep / horizontal
                maxLongGrade = math.max(maxLongGrade, grade)
                if heightStep > self.MAX_HEIGHT_STEP or grade > self.MAX_LONG_GRADE then
                    return nil, string.format("longitudinal grade %.2f", grade)
                end
            end
        end

        if not self:isCorridorPointClear(vehicle, temporary, width * 0.5 + 0.6) then
            return nil, "obstacle in right corridor"
        end

        table.insert(points, temporary)
        if holdPointIndex == nil and localDistance >= holdDistance then
            holdPointIndex = #points
        end
        previousPoint = temporary
        localDistance = localDistance + self.SAMPLE_STEP
    end

    local firstPoint = points[1]
    local lastPoint = points[#points]
    if firstPoint == nil or firstPoint.routeIndex == nil or lastPoint == nil or lastPoint.routeIndex == nil then
        return nil, "invalid generated route"
    end

    local grade = self:getGradeAtRouteDistance(vehicle, wayPoints, currentIndex, maneuverStartDistance)
    local currentSpeed = (vehicle.lastSpeedReal or 0) * 3600
    local score = maxCrossGrade * 350 + maxLongGrade * 220 + turnRadius * 1.5
        + offset * 8 + currentSpeed * 0.8 + length * 0.25
    if grade > 0.025 then
        score = score + 1000 + grade * 1000
    elseif grade < -0.025 then
        score = score - 180 + grade * 200
    end

    return {
        vehicle = vehicle,
        otherVehicle = otherVehicle,
        originalWayPoints = wayPoints,
        originalCurrentIndex = currentIndex,
        points = points,
        holdPointIndex = holdPointIndex or math.max(1, math.floor(#points / 2)),
        maneuverStartRouteIndex = firstPoint.routeIndex,
        rejoinRouteIndex = lastPoint.routeIndex,
        approachDistance = entryDistance,
        maneuverStartDistance = maneuverStartDistance,
        width = width,
        length = length,
        otherLength = otherLength,
        offset = offset,
        ramp = ramp,
        plateau = plateau,
        grade = grade,
        score = score
    }, nil
end

function ADDynamicYield:injectApproachCandidate(candidate)
    local drivePath = candidate.vehicle.ad.drivePathModule
    local newWayPoints = {}
    local prefixEnd = math.max(candidate.originalCurrentIndex - 1, candidate.maneuverStartRouteIndex - 1)

    for index = 1, prefixEnd do
        if candidate.originalWayPoints[index] ~= nil then
            table.insert(newWayPoints, candidate.originalWayPoints[index])
        end
    end

    local generatedStart = #newWayPoints + 1
    for _, point in ipairs(candidate.points) do
        table.insert(newWayPoints, point)
    end
    local holdIndex = generatedStart + candidate.holdPointIndex - 1

    for index = candidate.rejoinRouteIndex + 1, #candidate.originalWayPoints do
        table.insert(newWayPoints, candidate.originalWayPoints[index])
    end

    drivePath.wayPoints = newWayPoints
    drivePath.minDistanceToNextWp = math.huge
    drivePath:setCurrentWayPointIndex(math.min(candidate.originalCurrentIndex, generatedStart))
    drivePath.distanceToTarget = drivePath:getDistanceToLastWaypoint(40)
    if drivePath.setDirtyFlag ~= nil then
        drivePath:setDirtyFlag()
    end

    candidate.routeTable = newWayPoints
    candidate.generatedStartIndex = generatedStart
    candidate.holdRouteIndex = holdIndex
    candidate.generatedEndIndex = generatedStart + #candidate.points - 1
    candidate.holdPoint = newWayPoints[holdIndex]
    return true
end

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
        local now = g_time or 0
        if self.lastCandidateRejectLog == nil or now - self.lastCandidateRejectLog > 3000 then
            self.lastCandidateRejectLog = now
            self:log("Candidate rejected for %s / %s: %s / %s",
                dyaLabel(firstVehicle), dyaLabel(secondVehicle),
                tostring(firstReason), tostring(secondReason))
        end
        return false
    end

    local yieldCandidate
    local throughVehicle
    if firstCandidate ~= nil and secondCandidate == nil then
        yieldCandidate = firstCandidate
        throughVehicle = secondVehicle
    elseif secondCandidate ~= nil and firstCandidate == nil then
        yieldCandidate = secondCandidate
        throughVehicle = firstVehicle
    elseif firstCandidate.score <= secondCandidate.score then
        yieldCandidate = firstCandidate
        throughVehicle = secondVehicle
    else
        yieldCandidate = secondCandidate
        throughVehicle = firstVehicle
    end

    self:injectApproachCandidate(yieldCandidate)
    local pair = {
        id = self.nextPairId,
        yieldVehicle = yieldCandidate.vehicle,
        throughVehicle = throughVehicle,
        candidate = yieldCandidate,
        overlap = overlap,
        phase = "MANEUVER",
        createdAt = g_time or 0
    }
    self.nextPairId = self.nextPairId + 1
    table.insert(self.pairs, pair)
    self.vehiclePairs[pair.yieldVehicle] = {pair = pair, role = "yield", otherVehicle = pair.throughVehicle}
    self.vehiclePairs[pair.throughVehicle] = {pair = pair, role = "through", otherVehicle = pair.yieldVehicle}

    self:log(
        "Pair %d planned on approach: %s yields right after %.1fm, %s continues; shared=%.1fm offset=%.1fm grade=%.1f%%",
        pair.id,
        dyaLabel(pair.yieldVehicle),
        yieldCandidate.maneuverStartDistance,
        dyaLabel(pair.throughVehicle),
        overlap.sharedLength,
        yieldCandidate.offset,
        yieldCandidate.grade * 100
    )
    return true
end

local dyaOriginalLoadMap = ADDynamicYield.loadMap
function ADDynamicYield:loadMap(name)
    dyaOriginalLoadMap(self, name)
    self.enabled = true
    self.debugEnabled = true
    self:log(
        "%s approach planning active; pair range=%dm, manoeuvre starts %dm inside shared road",
        self.BUILD,
        self.MAX_PAIR_DISTANCE,
        self.ENTRY_BUFFER
    )
end
