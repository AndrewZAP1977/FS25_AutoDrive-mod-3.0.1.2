-- Experimental automatic right-side yielding for opposing AutoDrive traffic.
--
-- No permanent route nodes are created. The yielding vehicle receives a temporary
-- S-shaped list of pathfinder points, waits clear of the route, and rejoins the
-- original AutoDrive path after the opposing vehicle has passed.

if AutoDrive == nil or ADDrivePathModule == nil or ADStateModule == nil or ADCollisionDetectionModule == nil then
    Logging.error("[AD-DY] Required AutoDrive classes are unavailable")
    return
end

ADDynamicYield = {
    BUILD = "dynamic-yield-exp0.1",
    enabled = false,
    debugEnabled = false,
    scanElapsed = 0,
    nextPairId = 1,
    pairs = {},
    vehiclePairs = setmetatable({}, {__mode = "k"}),

    SCAN_INTERVAL_MS = 400,
    MIN_PAIR_DISTANCE = 55,
    MAX_PAIR_DISTANCE = 175,
    MIN_SHARED_NODES = 5,
    MIN_SHARED_LENGTH = 75,
    MAX_LOOKAHEAD_NODES = 80,

    CLEARANCE_MARGIN = 1.5,
    MIN_OFFSET = 3.5,
    MAX_OFFSET = 9.0,
    MIN_RAMP = 25,
    MAX_RAMP = 48,
    MIN_PLATEAU = 15,
    MAX_PLATEAU = 32,
    SAMPLE_STEP = 5,

    MAX_LONG_GRADE = 0.24,
    MAX_CROSS_GRADE = 0.22,
    MAX_HEIGHT_STEP = 1.25,

    APPROACH_DISTANCE = 45,
    FINAL_APPROACH_DISTANCE = 15,
    APPROACH_SPEED = 20,
    FINAL_APPROACH_SPEED = 10,
    HOLD_DISTANCE = 3,
    PASS_MARGIN = 7,
    PAIR_TIMEOUT_MS = 90000
}

local function dyClamp(value, minimum, maximum)
    return math.max(minimum, math.min(maximum, value))
end

local function dyDistance(x1, z1, x2, z2)
    return MathUtil.vector2Length(x2 - x1, z2 - z1)
end

local function dyNormalize(x, z)
    local length = MathUtil.vector2Length(x, z)
    if length < 0.0001 then
        return 0, 1
    end
    return x / length, z / length
end

local function dySmoothStep(value)
    value = dyClamp(value, 0, 1)
    return value * value * (3 - 2 * value)
end

local function dyLabel(vehicle)
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

function ADDynamicYield:log(message, ...)
    Logging.info("[AD-DY] " .. tostring(message), ...)
end

function ADDynamicYield:debug(message, ...)
    if self.debugEnabled then
        self:log(message, ...)
    end
end

function ADDynamicYield:isActiveVehicle(vehicle)
    return vehicle ~= nil
        and vehicle.components ~= nil
        and vehicle.components[1] ~= nil
        and vehicle.components[1].node ~= nil
        and entityExists(vehicle.components[1].node)
        and vehicle.ad ~= nil
        and vehicle.ad.stateModule ~= nil
        and vehicle.ad.drivePathModule ~= nil
        and vehicle.ad.specialDrivingModule ~= nil
        and vehicle.ad.stateModule:isActive()
        and not vehicle.spec_locomotive
end

function ADDynamicYield:getPosition(vehicle)
    local node = vehicle.components[1].node
    if vehicle.getAISteeringNode ~= nil then
        local steeringNode = vehicle:getAISteeringNode()
        if steeringNode ~= nil and entityExists(steeringNode) then
            node = steeringNode
        end
    end
    return getWorldTranslation(node)
end

function ADDynamicYield:getTerrainHeight(x, z)
    if g_currentMission ~= nil and g_currentMission.terrainRootNode ~= nil and getTerrainHeightAtWorldPos ~= nil then
        return getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, x, 0, z)
    end
    return 0
end

function ADDynamicYield:getRoute(vehicle)
    if not self:isActiveVehicle(vehicle) then
        return nil, nil
    end
    local wayPoints, currentIndex = vehicle.ad.drivePathModule:getWayPoints()
    if wayPoints == nil or currentIndex == nil or currentIndex < 1 or wayPoints[currentIndex] == nil then
        return nil, nil
    end
    return wayPoints, currentIndex
end

function ADDynamicYield:getWidth(vehicle)
    local width = vehicle.size ~= nil and vehicle.size.width or 3
    if AutoDrive.getVehicleDimensions ~= nil then
        local measured = select(1, AutoDrive.getVehicleDimensions(vehicle, false))
        if measured ~= nil and measured > 0 then
            width = math.max(width, measured)
        end
    end
    if AutoDrive.getAllImplements ~= nil then
        for _, implement in ipairs(AutoDrive.getAllImplements(vehicle, true) or {}) do
            if implement ~= nil and implement.size ~= nil then
                local implementWidth = implement.size.width or 0
                if AutoDrive.getVehicleDimensions ~= nil then
                    local measured = select(1, AutoDrive.getVehicleDimensions(implement, false))
                    if measured ~= nil and measured > 0 then
                        implementWidth = math.max(implementWidth, measured)
                    end
                end
                width = math.max(width, implementWidth)
            end
        end
    end
    return width
end

function ADDynamicYield:getLength(vehicle)
    local length = vehicle.size ~= nil and vehicle.size.length or 6
    if AutoDrive.getTractorTrainLength ~= nil then
        local measured = AutoDrive.getTractorTrainLength(vehicle, true, false)
        if measured ~= nil and measured > 0 then
            length = math.max(length, measured)
        end
    end
    return length
end

function ADDynamicYield:getTurnRadius(vehicle)
    if AutoDrive.getDriverRadius ~= nil then
        local radius = AutoDrive.getDriverRadius(vehicle)
        if radius ~= nil and radius > 0 then
            return radius
        end
    end
    return 8
end

function ADDynamicYield:routeDistance(wayPoints, firstIndex, lastIndex)
    local distance = 0
    if wayPoints == nil or firstIndex == nil or lastIndex == nil then
        return distance
    end
    for index = firstIndex, lastIndex - 1 do
        local first = wayPoints[index]
        local second = wayPoints[index + 1]
        if first == nil or second == nil then
            break
        end
        distance = distance + dyDistance(first.x, first.z, second.x, second.z)
    end
    return distance
end

function ADDynamicYield:sampleRoute(wayPoints, startIndex, wantedDistance, startX, startY, startZ)
    local previousX = startX
    local previousZ = startZ
    local remaining = math.max(wantedDistance, 0)
    for index = startIndex, #wayPoints do
        local point = wayPoints[index]
        if point ~= nil then
            local segmentLength = dyDistance(previousX, previousZ, point.x, point.z)
            if segmentLength > 0.001 then
                if remaining <= segmentLength then
                    local t = remaining / segmentLength
                    local x = previousX + (point.x - previousX) * t
                    local z = previousZ + (point.z - previousZ) * t
                    local tx, tz = dyNormalize(point.x - previousX, point.z - previousZ)
                    return {
                        x = x,
                        y = self:getTerrainHeight(x, z),
                        z = z,
                        tx = tx,
                        tz = tz,
                        routeIndex = index,
                        routeT = t
                    }
                end
                remaining = remaining - segmentLength
            end
            previousX = point.x
            previousZ = point.z
        end
    end
    return nil
end

function ADDynamicYield:getRouteHeading(vehicle, wayPoints, currentIndex)
    local x, y, z = self:getPosition(vehicle)
    local ahead = self:sampleRoute(wayPoints, currentIndex, 12, x, y, z)
    if ahead == nil then
        return nil
    end
    local tx, tz = dyNormalize(ahead.x - x, ahead.z - z)
    return {x = tx, z = tz}
end

function ADDynamicYield:isDualSharedPath(wayPoints, firstIndex, lastIndex)
    for index = firstIndex, lastIndex - 1 do
        local first = wayPoints[index]
        local second = wayPoints[index + 1]
        if first == nil or second == nil or first.id == nil or second.id == nil
            or not ADGraphManager:isDualRoad(first, second) then
            return false
        end
    end
    return true
end

function ADDynamicYield:findOpposingOverlap(firstVehicle, secondVehicle)
    local firstRoute, firstCurrent = self:getRoute(firstVehicle)
    local secondRoute, secondCurrent = self:getRoute(secondVehicle)
    if firstRoute == nil or secondRoute == nil then
        return nil
    end

    local firstHeading = self:getRouteHeading(firstVehicle, firstRoute, firstCurrent)
    local secondHeading = self:getRouteHeading(secondVehicle, secondRoute, secondCurrent)
    if firstHeading == nil or secondHeading == nil
        or firstHeading.x * secondHeading.x + firstHeading.z * secondHeading.z > -0.55 then
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
                    local sharedLength = self:routeDistance(firstRoute, firstStart, firstEnd)
                    if sharedLength >= self.MIN_SHARED_LENGTH
                        and self:isDualSharedPath(firstRoute, firstStart, firstEnd)
                        and (best == nil or sharedLength > best.sharedLength) then
                        best = {
                            firstRoute = firstRoute,
                            firstCurrent = firstCurrent,
                            firstStart = firstStart,
                            firstEnd = firstEnd,
                            secondRoute = secondRoute,
                            secondCurrent = secondCurrent,
                            secondStart = secondEnd - count + 1,
                            secondEnd = secondEnd,
                            sharedLength = sharedLength,
                            nodeCount = count
                        }
                    end
                end
            end
        end
    end
    return best
end

function ADDynamicYield:getForwardGrade(vehicle, wayPoints, currentIndex)
    local x, y, z = self:getPosition(vehicle)
    local ahead = self:sampleRoute(wayPoints, currentIndex, 25, x, y, z)
    if ahead == nil then
        return 0
    end
    return (ahead.y - self:getTerrainHeight(x, z)) / math.max(dyDistance(x, z, ahead.x, ahead.z), 1)
end

function ADDynamicYield:dynamicYieldOverlapCallback(transformId)
    if self.overlapBlocked or transformId == nil or transformId == 0 then
        return
    end
    if g_currentMission ~= nil and transformId == g_currentMission.terrainRootNode then
        return
    end
    local object = g_currentMission ~= nil and g_currentMission.nodeToObject[transformId] or nil
    if object ~= nil and self.overlapVehicle ~= nil then
        if object == self.overlapVehicle
            or (AutoDrive.checkIsConnected ~= nil and AutoDrive:checkIsConnected(self.overlapVehicle, object)) then
            return
        end
    end
    self.overlapBlocked = true
end

function ADDynamicYield:isCorridorPointClear(vehicle, point, halfWidth)
    if overlapBox == nil or AutoDrive.collisionMaskTerrain == nil then
        return true
    end
    self.overlapVehicle = vehicle
    self.overlapBlocked = false
    local extent = math.max(halfWidth, 2.5)
    overlapBox(point.x, point.y + 2.5, point.z, 0, 0, 0,
        extent, 2.5, extent, "dynamicYieldOverlapCallback", self,
        AutoDrive.collisionMaskTerrain, true, true, true, true)
    self.overlapVehicle = nil
    return not self.overlapBlocked
end

function ADDynamicYield:buildCandidate(vehicle, otherVehicle)
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
    local offset = dyClamp(width * 0.5 + otherWidth * 0.5 + self.CLEARANCE_MARGIN,
        self.MIN_OFFSET, self.MAX_OFFSET)
    local ramp = dyClamp(math.max(self.MIN_RAMP, turnRadius * 2.2, length * 0.32),
        self.MIN_RAMP, self.MAX_RAMP)
    local plateau = dyClamp(self.MIN_PLATEAU + length * 0.18,
        self.MIN_PLATEAU, self.MAX_PLATEAU)
    local totalLength = ramp * 2 + plateau
    local holdDistance = ramp + plateau * 0.5

    if self:routeDistance(wayPoints, currentIndex, #wayPoints) < totalLength + 10 then
        return nil, "insufficient future route"
    end

    local points = {}
    local holdPointIndex = nil
    local previousPoint = nil
    local maxLongGrade = 0
    local maxCrossGrade = 0
    local distance = self.SAMPLE_STEP

    while distance <= totalLength + 0.001 do
        local center = self:sampleRoute(wayPoints, currentIndex, distance, x, y, z)
        if center == nil then
            return nil, "route sampling failed"
        end

        local profile
        if distance < ramp then
            profile = dySmoothStep(distance / ramp)
        elseif distance <= ramp + plateau then
            profile = 1
        else
            profile = 1 - dySmoothStep((distance - ramp - plateau) / ramp)
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
            local horizontal = dyDistance(previousPoint.x, previousPoint.z, temporary.x, temporary.z)
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
        if holdPointIndex == nil and distance >= holdDistance then
            holdPointIndex = #points
        end
        previousPoint = temporary
        distance = distance + self.SAMPLE_STEP
    end

    local lastPoint = points[#points]
    if lastPoint == nil or lastPoint.routeIndex == nil then
        return nil, "no rejoin point"
    end

    local grade = self:getForwardGrade(vehicle, wayPoints, currentIndex)
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
        rejoinRouteIndex = lastPoint.routeIndex,
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

function ADDynamicYield:injectCandidate(candidate)
    local drivePath = candidate.vehicle.ad.drivePathModule
    local newWayPoints = {}
    for index = 1, candidate.originalCurrentIndex - 1 do
        table.insert(newWayPoints, candidate.originalWayPoints[index])
    end
    local generatedStart = #newWayPoints + 1
    for _, point in ipairs(candidate.points) do
        table.insert(newWayPoints, point)
    end
    local holdIndex = generatedStart + candidate.holdPointIndex - 1
    for index = math.min(candidate.rejoinRouteIndex + 1, #candidate.originalWayPoints + 1), #candidate.originalWayPoints do
        table.insert(newWayPoints, candidate.originalWayPoints[index])
    end

    drivePath.wayPoints = newWayPoints
    drivePath.minDistanceToNextWp = math.huge
    drivePath:setCurrentWayPointIndex(generatedStart)
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
    local firstCandidate, firstReason = self:buildCandidate(firstVehicle, secondVehicle)
    local secondCandidate, secondReason = self:buildCandidate(secondVehicle, firstVehicle)
    if firstCandidate == nil and secondCandidate == nil then
        self:debug("Rejected %s / %s: %s / %s", dyLabel(firstVehicle), dyLabel(secondVehicle),
            tostring(firstReason), tostring(secondReason))
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

    self:injectCandidate(yieldCandidate)
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

    self:log("Pair %d: %s yields right, %s continues; shared=%.1fm offset=%.1fm grade=%.1f%% score=%.1f",
        pair.id, dyLabel(pair.yieldVehicle), dyLabel(pair.throughVehicle), overlap.sharedLength,
        yieldCandidate.offset, yieldCandidate.grade * 100, yieldCandidate.score)
    return true
end

function ADDynamicYield:getPairState(vehicle)
    return vehicle ~= nil and self.vehiclePairs[vehicle] or nil
end

function ADDynamicYield:distanceToIndex(vehicle, routeTable, targetIndex)
    local _, currentIndex = vehicle.ad.drivePathModule:getWayPoints()
    if currentIndex == nil or currentIndex > targetIndex then
        return 0
    end
    local x, _, z = self:getPosition(vehicle)
    local distance = 0
    local previousX = x
    local previousZ = z
    for index = currentIndex, targetIndex do
        local point = routeTable[index]
        if point ~= nil then
            distance = distance + dyDistance(previousX, previousZ, point.x, point.z)
            previousX = point.x
            previousZ = point.z
        end
    end
    return distance
end

function ADDynamicYield:shouldHold(vehicle)
    local state = self:getPairState(vehicle)
    if state == nil or state.role ~= "yield" or state.pair == nil or state.pair.phase == "RELEASED" then
        return false
    end
    local pair = state.pair
    local candidate = pair.candidate
    local wayPoints, currentIndex = vehicle.ad.drivePathModule:getWayPoints()
    if wayPoints ~= candidate.routeTable or currentIndex == nil then
        self:clearPair(pair, "yield route replaced")
        return false
    end
    local distance = self:distanceToIndex(vehicle, candidate.routeTable, candidate.holdRouteIndex)
    if pair.phase == "MANEUVER" and (currentIndex >= candidate.holdRouteIndex or distance <= self.HOLD_DISTANCE) then
        pair.phase = "HOLD"
        self:log("Pair %d holding %s %.1fm right of route", pair.id, dyLabel(vehicle), candidate.offset)
    end
    return pair.phase == "HOLD"
end

function ADDynamicYield:getDynamicSpeedLimit(vehicle)
    local state = self:getPairState(vehicle)
    if state == nil or state.role ~= "yield" or state.pair == nil or state.pair.phase == "RELEASED" then
        return nil
    end
    local candidate = state.pair.candidate
    local distance = self:distanceToIndex(vehicle, candidate.routeTable, candidate.holdRouteIndex)
    if distance <= self.FINAL_APPROACH_DISTANCE then
        return self.FINAL_APPROACH_SPEED
    elseif distance <= self.APPROACH_DISTANCE then
        return self.APPROACH_SPEED
    end
    return nil
end

function ADDynamicYield:throughPassed(pair)
    local hold = pair.candidate ~= nil and pair.candidate.holdPoint or nil
    if hold == nil or not self:isActiveVehicle(pair.throughVehicle) then
        return false
    end
    local x, _, z = self:getPosition(pair.throughVehicle)
    local signedDistance = (x - hold.x) * hold.tx + (z - hold.z) * hold.tz
    local required = pair.candidate.length * 0.5 + pair.candidate.otherLength * 0.5 + self.PASS_MARGIN
    return signedDistance < -required
end

function ADDynamicYield:clearPair(pair, reason)
    if pair == nil or pair.cleared then
        return
    end
    pair.cleared = true
    pair.phase = "RELEASED"
    self.vehiclePairs[pair.yieldVehicle] = nil
    self.vehiclePairs[pair.throughVehicle] = nil
    self:log("Pair %d cleared: %s", pair.id, tostring(reason or "complete"))
end

function ADDynamicYield:updatePairs()
    for index = #self.pairs, 1, -1 do
        local pair = self.pairs[index]
        local remove = false
        if pair.cleared then
            remove = true
        elseif not self:isActiveVehicle(pair.yieldVehicle) or not self:isActiveVehicle(pair.throughVehicle) then
            self:clearPair(pair, "vehicle inactive")
            remove = true
        elseif (g_time or 0) - pair.createdAt > self.PAIR_TIMEOUT_MS then
            self:clearPair(pair, "timeout; yielding vehicle released")
            remove = true
        else
            local candidate = pair.candidate
            local wayPoints, currentIndex = pair.yieldVehicle.ad.drivePathModule:getWayPoints()
            if wayPoints ~= candidate.routeTable or currentIndex == nil then
                self:clearPair(pair, "yield route changed")
                remove = true
            elseif pair.phase == "HOLD" and self:throughPassed(pair) then
                pair.phase = "RELEASED"
                self:log("Pair %d releasing %s after %s passed", pair.id,
                    dyLabel(pair.yieldVehicle), dyLabel(pair.throughVehicle))
            elseif pair.phase == "RELEASED" and currentIndex > candidate.generatedEndIndex then
                self:clearPair(pair, "yield vehicle returned to route")
                remove = true
            end
        end
        if remove then
            table.remove(self.pairs, index)
        end
    end
end

function ADDynamicYield:scanForPairs()
    if not self.enabled or g_server == nil then
        return
    end
    local vehicles = {}
    for _, vehicle in pairs(AutoDrive.getAllVehicles() or {}) do
        if self:isActiveVehicle(vehicle) and self:getPairState(vehicle) == nil
            and vehicle.ad.drivePathModule:isOnRoadNetwork() then
            table.insert(vehicles, vehicle)
        end
    end

    for firstIndex = 1, #vehicles - 1 do
        local first = vehicles[firstIndex]
        local firstX, _, firstZ = self:getPosition(first)
        for secondIndex = firstIndex + 1, #vehicles do
            local second = vehicles[secondIndex]
            local secondX, _, secondZ = self:getPosition(second)
            local distance = dyDistance(firstX, firstZ, secondX, secondZ)
            if distance >= self.MIN_PAIR_DISTANCE and distance <= self.MAX_PAIR_DISTANCE then
                local overlap = self:findOpposingOverlap(first, second)
                if overlap ~= nil and self:createPair(first, second, overlap) then
                    return
                end
            end
        end
    end
end

function ADDynamicYield:update(dt)
    if not self.loaded or g_server == nil then
        return
    end
    self:updatePairs()
    self.scanElapsed = self.scanElapsed + dt
    if self.scanElapsed >= self.SCAN_INTERVAL_MS then
        self.scanElapsed = 0
        self:scanForPairs()
    end
end

function ADDynamicYield:loadMap()
    self.loaded = true
    self.scanElapsed = 0
    self.pairs = {}
    self.vehiclePairs = setmetatable({}, {__mode = "k"})
    addConsoleCommand("adDynamicYield", "Dynamic right-side yielding: on|off|status", "consoleToggle", self)
    addConsoleCommand("adDynamicYieldDebug", "Dynamic-yield debug: on|off", "consoleDebug", self)
    self:log("%s loaded; disabled by default. Enable with: adDynamicYield on", self.BUILD)
end

function ADDynamicYield:deleteMap()
    removeConsoleCommand("adDynamicYield")
    removeConsoleCommand("adDynamicYieldDebug")
    self.loaded = false
    self.pairs = {}
    self.vehiclePairs = setmetatable({}, {__mode = "k"})
end

function ADDynamicYield:consoleToggle(value)
    local normalized = value ~= nil and string.lower(tostring(value)) or "status"
    if normalized == "on" or normalized == "1" or normalized == "true" then
        self.enabled = true
    elseif normalized == "off" or normalized == "0" or normalized == "false" then
        self.enabled = false
        for _, pair in ipairs(self.pairs) do
            self:clearPair(pair, "feature disabled")
        end
    end
    self:log("enabled=%s activePairs=%d scan=%dms range=%d-%dm", tostring(self.enabled),
        #self.pairs, self.SCAN_INTERVAL_MS, self.MIN_PAIR_DISTANCE, self.MAX_PAIR_DISTANCE)
end

function ADDynamicYield:consoleDebug(value)
    local normalized = value ~= nil and string.lower(tostring(value)) or ""
    if normalized == "on" or normalized == "1" or normalized == "true" then
        self.debugEnabled = true
    elseif normalized == "off" or normalized == "0" or normalized == "false" then
        self.debugEnabled = false
    end
    self:log("debug=%s", tostring(self.debugEnabled))
end

local dyOriginalFollowWaypoints = ADDrivePathModule.followWaypoints
function ADDrivePathModule:followWaypoints(dt)
    if ADDynamicYield ~= nil and ADDynamicYield:shouldHold(self.vehicle) then
        self.vehicle.ad.specialDrivingModule:stopVehicle()
        self.vehicle.ad.specialDrivingModule:update(dt)
        return
    end
    return dyOriginalFollowWaypoints(self, dt)
end

local dyOriginalGetSpeedLimit = ADStateModule.getSpeedLimit
function ADStateModule:getSpeedLimit()
    local speedLimit = dyOriginalGetSpeedLimit(self)
    if ADDynamicYield ~= nil then
        local dynamicLimit = ADDynamicYield:getDynamicSpeedLimit(self.vehicle)
        if dynamicLimit ~= nil then
            speedLimit = math.min(speedLimit, dynamicLimit)
        end
    end
    return speedLimit
end

if ADStateModule.getFieldSpeedLimit ~= nil then
    local dyOriginalGetFieldSpeedLimit = ADStateModule.getFieldSpeedLimit
    function ADStateModule:getFieldSpeedLimit()
        local speedLimit = dyOriginalGetFieldSpeedLimit(self)
        if ADDynamicYield ~= nil then
            local dynamicLimit = ADDynamicYield:getDynamicSpeedLimit(self.vehicle)
            if dynamicLimit ~= nil then
                speedLimit = math.min(speedLimit, dynamicLimit)
            end
        end
        return speedLimit
    end
end

local dyOriginalDetectAdTrafficOnRoute = ADCollisionDetectionModule.detectAdTrafficOnRoute
function ADCollisionDetectionModule:detectAdTrafficOnRoute()
    local detected = dyOriginalDetectAdTrafficOnRoute(self)
    if not detected or ADDynamicYield == nil then
        return detected
    end
    local state = ADDynamicYield:getPairState(self.vehicle)
    if state ~= nil and state.otherVehicle ~= nil and self.trafficVehicle == state.otherVehicle then
        self.trafficVehicle = nil
        return false
    end
    return detected
end

addModEventListener(ADDynamicYield)
