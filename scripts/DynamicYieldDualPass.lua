-- Coordinated two-vehicle right-side passing.
--
-- Preferred mode on a sufficiently long two-way road:
--   1. reserve the opposing pair early;
--   2. choose one common meeting station on the shared route;
--   3. move both vehicles to their own physical right side;
--   4. synchronize arrival at the meeting station without a planned stop;
--   5. let both trains clear each other and independently rejoin the centreline.
--
-- If either narrow right corridor is unavailable, the already-tested SINGLE
-- controller is called. If SINGLE also fails, its existing SERIAL fallback is
-- retained unchanged.

if ADDynamicYield == nil then
    Logging.error("[AD-DY] Dual-pass controller could not load")
    return
end

ADDynamicYield.BUILD = "dynamic-yield-dual-exp0.1"
ADDynamicYield.DUAL_HALF_GAP = 0.40
ADDynamicYield.DUAL_TOTAL_GAP = ADDynamicYield.DUAL_HALF_GAP * 2
ADDynamicYield.DUAL_PRE_MEETING_MARGIN = 5
ADDynamicYield.DUAL_CLEAR_MARGIN = 4
ADDynamicYield.DUAL_MIN_PLATEAU = 16
ADDynamicYield.DUAL_MIN_RAMP = 9
ADDynamicYield.DUAL_MAX_RAMP = 22
ADDynamicYield.DUAL_PASS_SPEED = 18
ADDynamicYield.DUAL_CLEAR_SPEED = 18
ADDynamicYield.DUAL_MIN_ROLL_SPEED = 4
ADDynamicYield.DUAL_EMERGENCY_LEAD = 8
ADDynamicYield.DUAL_TRIGGER_DISTANCE = ADDynamicYield.MANEUVER_TRIGGER_ROUTE_DISTANCE or 150
ADDynamicYield.DUAL_CHECK_INTERVAL_MS = 400
ADDynamicYield.DUAL_LOG_INTERVAL_MS = 2500
ADDynamicYield.DUAL_TIMEOUT_MS = 120000

local function dyduClamp(value, minimum, maximum)
    return math.max(minimum, math.min(maximum, value))
end

local function dyduDistance(x1, z1, x2, z2)
    return MathUtil.vector2Length(x2 - x1, z2 - z1)
end

local function dyduNormalize(x, z)
    local length = MathUtil.vector2Length(x, z)
    if length < 0.0001 then
        return 1, 0
    end
    return x / length, z / length
end

local function dyduSmoothStep(value)
    value = dyduClamp(value, 0, 1)
    return value * value * (3 - 2 * value)
end

local function dyduLabel(vehicle)
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

local function dyduGetSteeringNode(vehicle)
    if vehicle ~= nil and vehicle.getAISteeringNode ~= nil then
        local node = vehicle:getAISteeringNode()
        if node ~= nil and entityExists(node) then
            return node
        end
    end
    if vehicle ~= nil and vehicle.components ~= nil and vehicle.components[1] ~= nil then
        local node = vehicle.components[1].node
        if node ~= nil and entityExists(node) then
            return node
        end
    end
    return nil
end

local function dyduDistanceToIndex(manager, vehicle, route, targetIndex)
    if vehicle == nil or route == nil or targetIndex == nil then
        return math.huge
    end
    local currentRoute, currentIndex = manager:getRoute(vehicle)
    if currentRoute ~= route or currentIndex == nil then
        return math.huge
    end
    if currentIndex > targetIndex then
        return 0
    end

    local x, _, z = manager:getPosition(vehicle)
    local distance = 0
    local previousX = x
    local previousZ = z
    for index = currentIndex, targetIndex do
        local point = route[index]
        if point == nil then
            return math.huge
        end
        distance = distance + dyduDistance(previousX, previousZ, point.x, point.z)
        previousX = point.x
        previousZ = point.z
    end
    return distance
end

local function dyduDistancePastIndex(manager, vehicle, route, entryIndex)
    if vehicle == nil or route == nil or entryIndex == nil then
        return 0
    end
    local currentRoute, currentIndex = manager:getRoute(vehicle)
    if currentRoute ~= route or currentIndex == nil or currentIndex <= entryIndex then
        return 0
    end

    local distance = 0
    for index = entryIndex, currentIndex - 2 do
        local first = route[index]
        local second = route[index + 1]
        if first == nil or second == nil then
            return distance
        end
        distance = distance + dyduDistance(first.x, first.z, second.x, second.z)
    end

    local previous = route[currentIndex - 1]
    if previous ~= nil then
        local x, _, z = manager:getPosition(vehicle)
        distance = distance + dyduDistance(previous.x, previous.z, x, z)
    end
    return distance
end

local function dyduSignedDistanceToEntry(manager, vehicle, route, entryIndex)
    local currentRoute, currentIndex = manager:getRoute(vehicle)
    if currentRoute ~= route or currentIndex == nil then
        return math.huge
    end
    if currentIndex <= entryIndex then
        return dyduDistanceToIndex(manager, vehicle, route, entryIndex)
    end
    return -dyduDistancePastIndex(manager, vehicle, route, entryIndex)
end

local function dyduCurrentOverlap(manager, firstVehicle, secondVehicle, overlap)
    if overlap == nil then
        return nil
    end

    local firstRoute, firstCurrent = manager:getRoute(firstVehicle)
    local secondRoute, secondCurrent = manager:getRoute(secondVehicle)
    if firstRoute ~= overlap.firstRoute or secondRoute ~= overlap.secondRoute
        or firstCurrent == nil or secondCurrent == nil then
        return nil
    end

    local firstSigned = dyduSignedDistanceToEntry(
        manager,
        firstVehicle,
        firstRoute,
        overlap.firstStart
    )
    local secondSigned = dyduSignedDistanceToEntry(
        manager,
        secondVehicle,
        secondRoute,
        overlap.secondStart
    )
    if firstSigned == math.huge or secondSigned == math.huge then
        return nil
    end

    local sharedLength = overlap.sharedLength or 0
    local firstX, _, firstZ = manager:getPosition(firstVehicle)
    local secondX, _, secondZ = manager:getPosition(secondVehicle)
    return {
        firstRoute = firstRoute,
        firstCurrent = firstCurrent,
        firstStart = overlap.firstStart,
        firstEnd = overlap.firstEnd,
        secondRoute = secondRoute,
        secondCurrent = secondCurrent,
        secondStart = overlap.secondStart,
        secondEnd = overlap.secondEnd,
        sharedLength = sharedLength,
        nodeCount = overlap.nodeCount,
        firstSigned = firstSigned,
        secondSigned = secondSigned,
        firstApproachDistance = math.max(firstSigned, 0),
        secondApproachDistance = math.max(secondSigned, 0),
        firstAvailableShared = math.max(0, sharedLength + math.min(firstSigned, 0)),
        secondAvailableShared = math.max(0, sharedLength + math.min(secondSigned, 0)),
        firstCoordinate = -firstSigned,
        secondCoordinate = sharedLength + secondSigned,
        routeSeparation = math.max(0, sharedLength + firstSigned + secondSigned),
        worldSeparation = dyduDistance(firstX, firstZ, secondX, secondZ)
    }
end

local function dyduBuildGeometry(manager, vehicle, otherVehicle)
    local width = manager:getWidth(vehicle)
    local trainLength = manager:getLength(vehicle)
    local otherLength = manager:getLength(otherVehicle)
    local turnRadius = manager:getTurnRadius(vehicle)
    local offset = width * 0.5 + manager.DUAL_HALF_GAP
    local curvatureRamp = math.sqrt(math.max(0, 6 * offset * math.max(turnRadius, 1))) * 1.15
    local ramp = dyduClamp(
        math.max(manager.DUAL_MIN_RAMP, curvatureRamp),
        manager.DUAL_MIN_RAMP,
        manager.DUAL_MAX_RAMP
    )
    local clearAfterMeeting = (trainLength + otherLength) * 0.5 + manager.DUAL_CLEAR_MARGIN
    local plateau = math.max(
        manager.DUAL_MIN_PLATEAU,
        manager.DUAL_PRE_MEETING_MARGIN + clearAfterMeeting
    )
    local totalLength = ramp * 2 + plateau

    return {
        width = width,
        trainLength = trainLength,
        otherLength = otherLength,
        turnRadius = turnRadius,
        offset = offset,
        ramp = ramp,
        clearAfterMeeting = clearAfterMeeting,
        plateau = plateau,
        totalLength = totalLength,
        meetingLocalDistance = ramp + manager.DUAL_PRE_MEETING_MARGIN,
        postMeetingDistance = clearAfterMeeting + ramp + manager.REJOIN_MARGIN
    }
end

local function dyduChooseMeetingPlan(manager, firstVehicle, secondVehicle, current)
    local firstGeometry = dyduBuildGeometry(manager, firstVehicle, secondVehicle)
    local secondGeometry = dyduBuildGeometry(manager, secondVehicle, firstVehicle)
    local sharedLength = current.sharedLength

    -- Coordinate system: zero is the first vehicle's entry; sharedLength is the
    -- second vehicle's entry. Both vehicles move toward the selected coordinate.
    local lower = math.max(
        manager.ENTRY_BUFFER + firstGeometry.ramp + manager.DUAL_PRE_MEETING_MARGIN,
        secondGeometry.postMeetingDistance
    )
    local upper = math.min(
        sharedLength - firstGeometry.postMeetingDistance,
        sharedLength - (manager.ENTRY_BUFFER + secondGeometry.ramp + manager.DUAL_PRE_MEETING_MARGIN)
    )

    if lower > upper then
        return nil, string.format(
            "shared road has no dual meeting interval (lower %.1fm, upper %.1fm)",
            lower,
            upper
        )
    end

    local desiredMeeting = (current.firstCoordinate + current.secondCoordinate) * 0.5
    local meetingCoordinate = dyduClamp(desiredMeeting, lower, upper)
    local firstMeetingDistance = meetingCoordinate - current.firstCoordinate
    local secondMeetingDistance = current.secondCoordinate - meetingCoordinate

    if firstMeetingDistance <= firstGeometry.meetingLocalDistance
        or secondMeetingDistance <= secondGeometry.meetingLocalDistance then
        return nil, string.format(
            "vehicles are too close to establish dual lanes (%.1fm / %.1fm)",
            firstMeetingDistance,
            secondMeetingDistance
        )
    end

    return {
        meetingCoordinate = meetingCoordinate,
        intervalLower = lower,
        intervalUpper = upper,
        firstMeetingDistance = firstMeetingDistance,
        secondMeetingDistance = secondMeetingDistance,
        firstGeometry = firstGeometry,
        secondGeometry = secondGeometry
    }, nil
end

local function dyduBuildCandidate(manager, vehicle, otherVehicle, route, currentIndex,
    approachDistance, availableShared, distanceToMeeting, geometry)
    if route == nil or currentIndex == nil then
        return nil, "no route"
    end

    local steeringNode = dyduGetSteeringNode(vehicle)
    if steeringNode == nil or localDirectionToWorld == nil then
        return nil, "cannot determine physical right axis"
    end

    local startX, startY, startZ = manager:getPosition(vehicle)
    local startDistance = distanceToMeeting - geometry.meetingLocalDistance
    local earliestStart = math.max(0, approachDistance) + manager.ENTRY_BUFFER
    if startDistance + 0.1 < earliestStart then
        return nil, string.format(
            "dual lane would begin before shared-road entry (start %.1fm, earliest %.1fm)",
            startDistance,
            earliestStart
        )
    end

    local distanceToSharedExit = math.max(0, approachDistance) + math.max(0, availableShared)
    local requiredToExit = startDistance + geometry.totalLength + manager.REJOIN_MARGIN
    if requiredToExit > distanceToSharedExit then
        return nil, string.format(
            "dual route too short after meeting (need %.1fm, have %.1fm)",
            requiredToExit,
            distanceToSharedExit
        )
    end

    if manager:routeDistance(route, currentIndex, #route) < requiredToExit + 2 then
        return nil, "insufficient future route for dual lane"
    end

    local firstCenter = manager:sampleRoute(
        route,
        currentIndex,
        startDistance,
        startX,
        startY,
        startZ
    )
    if firstCenter == nil then
        return nil, "cannot sample dual-lane start"
    end

    -- GIANTS vehicle nodes use local -X as physical right.
    local physicalRightX, _, physicalRightZ = localDirectionToWorld(steeringNode, -1, 0, 0)
    physicalRightX, physicalRightZ = dyduNormalize(physicalRightX, physicalRightZ)
    local baseRightX, baseRightZ = dyduNormalize(firstCenter.tz, -firstCenter.tx)
    local rightDot = baseRightX * physicalRightX + baseRightZ * physicalRightZ
    local sideSign = rightDot >= 0 and 1 or -1

    local points = {}
    local meetingPointIndex = nil
    local previousPoint = nil
    local maxCrossGrade = 0
    local maxLongGrade = 0
    local localDistance = 0

    while true do
        local center = manager:sampleRoute(
            route,
            currentIndex,
            startDistance + localDistance,
            startX,
            startY,
            startZ
        )
        if center == nil then
            return nil, "dual route sampling failed"
        end

        local profile
        if localDistance < geometry.ramp then
            profile = dyduSmoothStep(localDistance / geometry.ramp)
        elseif localDistance <= geometry.ramp + geometry.plateau then
            profile = 1
        else
            profile = 1 - dyduSmoothStep(
                (localDistance - geometry.ramp - geometry.plateau) / geometry.ramp
            )
        end

        local lateralOffset = geometry.offset * profile
        local rightX, rightZ = dyduNormalize(center.tz * sideSign, -center.tx * sideSign)
        local pointX = center.x + rightX * lateralOffset
        local pointZ = center.z + rightZ * lateralOffset
        local pointY = manager:getTerrainHeight(pointX, pointZ)
        local crossGrade = lateralOffset > 1 and math.abs(pointY - center.y) / lateralOffset or 0
        maxCrossGrade = math.max(maxCrossGrade, crossGrade)
        if crossGrade > manager.MAX_CROSS_GRADE then
            return nil, string.format("dual right-side cross grade %.2f", crossGrade)
        end

        local point = {
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
            local horizontal = dyduDistance(previousPoint.x, previousPoint.z, point.x, point.z)
            if horizontal > 0.1 then
                local heightStep = math.abs(point.y - previousPoint.y)
                local grade = heightStep / horizontal
                maxLongGrade = math.max(maxLongGrade, grade)
                if heightStep > manager.MAX_HEIGHT_STEP or grade > manager.MAX_LONG_GRADE then
                    return nil, string.format("dual right-side longitudinal grade %.2f", grade)
                end
            end
        end

        if not manager:isCorridorPointClear(vehicle, point, geometry.width * 0.5 + 0.5) then
            return nil, "obstacle in narrow dual right corridor"
        end

        table.insert(points, point)
        if meetingPointIndex == nil and localDistance >= geometry.meetingLocalDistance then
            meetingPointIndex = #points
        end
        previousPoint = point

        if localDistance >= geometry.totalLength - 0.001 then
            break
        end
        localDistance = math.min(localDistance + manager.SAMPLE_STEP, geometry.totalLength)
    end

    local firstPoint = points[1]
    local lastPoint = points[#points]
    if firstPoint == nil or lastPoint == nil
        or firstPoint.routeIndex == nil or lastPoint.routeIndex == nil then
        return nil, "invalid generated dual route"
    end

    return {
        vehicle = vehicle,
        otherVehicle = otherVehicle,
        originalWayPoints = route,
        originalCurrentIndex = currentIndex,
        points = points,
        holdPointIndex = meetingPointIndex or math.max(1, math.floor(#points * 0.5)),
        meetingPointIndex = meetingPointIndex or math.max(1, math.floor(#points * 0.5)),
        maneuverStartRouteIndex = firstPoint.routeIndex,
        rejoinRouteIndex = lastPoint.routeIndex,
        approachDistance = approachDistance,
        maneuverStartDistance = startDistance,
        width = geometry.width,
        length = geometry.trainLength,
        trainLength = geometry.trainLength,
        otherLength = geometry.otherLength,
        offset = geometry.offset,
        ramp = geometry.ramp,
        plateau = geometry.plateau,
        totalLength = geometry.totalLength,
        meetingDistance = distanceToMeeting,
        meetingLocalDistance = geometry.meetingLocalDistance,
        clearAfterMeeting = geometry.clearAfterMeeting,
        maxCrossGrade = maxCrossGrade,
        maxLongGrade = maxLongGrade,
        sideSign = sideSign,
        rightAxisDot = rightDot,
        sideName = "RIGHT",
        sideAxis = "LOCAL_NEGATIVE_X"
    }, nil
end

local function dyduBuildPlan(manager, firstVehicle, secondVehicle, current)
    local meetingPlan, meetingReason = dyduChooseMeetingPlan(
        manager,
        firstVehicle,
        secondVehicle,
        current
    )
    if meetingPlan == nil then
        return nil, meetingReason
    end

    local firstCandidate, firstReason = dyduBuildCandidate(
        manager,
        firstVehicle,
        secondVehicle,
        current.firstRoute,
        current.firstCurrent,
        current.firstApproachDistance,
        current.firstAvailableShared,
        meetingPlan.firstMeetingDistance,
        meetingPlan.firstGeometry
    )
    if firstCandidate == nil then
        return nil, string.format("%s: %s", dyduLabel(firstVehicle), tostring(firstReason))
    end

    local secondCandidate, secondReason = dyduBuildCandidate(
        manager,
        secondVehicle,
        firstVehicle,
        current.secondRoute,
        current.secondCurrent,
        current.secondApproachDistance,
        current.secondAvailableShared,
        meetingPlan.secondMeetingDistance,
        meetingPlan.secondGeometry
    )
    if secondCandidate == nil then
        return nil, string.format("%s: %s", dyduLabel(secondVehicle), tostring(secondReason))
    end

    meetingPlan.firstCandidate = firstCandidate
    meetingPlan.secondCandidate = secondCandidate
    return meetingPlan, nil
end

local dyduOriginalCreatePair = ADDynamicYield.createPair
function ADDynamicYield:createPair(firstVehicle, secondVehicle, overlap)
    local current = dyduCurrentOverlap(self, firstVehicle, secondVehicle, overlap)
    if current ~= nil then
        local plan, reason = dyduBuildPlan(self, firstVehicle, secondVehicle, current)
        if plan ~= nil then
            local pair = {
                id = self.nextPairId,
                mode = "DUAL",
                phase = "ARMED",
                firstVehicle = firstVehicle,
                secondVehicle = secondVehicle,
                yieldVehicle = firstVehicle,
                throughVehicle = secondVehicle,
                overlap = overlap,
                previewPlan = plan,
                candidate = plan.firstCandidate,
                armedAt = g_time or 0,
                createdAt = g_time or 0,
                nextDualCheck = 0,
                dualSync = {}
            }
            self.nextPairId = self.nextPairId + 1
            table.insert(self.pairs, pair)
            self.vehiclePairs[firstVehicle] = {
                pair = pair,
                role = "dual",
                otherVehicle = secondVehicle
            }
            self.vehiclePairs[secondVehicle] = {
                pair = pair,
                role = "dual",
                otherVehicle = firstVehicle
            }

            self:log(
                "Pair %d hybrid mode=DUAL reserved: %s offset=%.2fm, %s offset=%.2fm; gap=%.2fm separation=%.1fm trigger=%dm",
                pair.id,
                dyduLabel(firstVehicle),
                plan.firstCandidate.offset,
                dyduLabel(secondVehicle),
                plan.secondCandidate.offset,
                self.DUAL_TOTAL_GAP,
                current.routeSeparation,
                self.DUAL_TRIGGER_DISTANCE
            )
            self:log(
                "Pair %d dual preview: meeting=%.1fm in shared interval %.1f..%.1fm; ramps=%.1f/%.1fm plateaus=%.1f/%.1fm",
                pair.id,
                plan.meetingCoordinate,
                plan.intervalLower,
                plan.intervalUpper,
                plan.firstCandidate.ramp,
                plan.secondCandidate.ramp,
                plan.firstCandidate.plateau,
                plan.secondCandidate.plateau
            )
            return true
        end

        self:log(
            "Dual mode unavailable for %s / %s: %s; trying SINGLE",
            dyduLabel(firstVehicle),
            dyduLabel(secondVehicle),
            tostring(reason)
        )
    end

    return dyduOriginalCreatePair(self, firstVehicle, secondVehicle, overlap)
end

local dyduOriginalActivateArmedPair = ADDynamicYield.activateArmedPair
function ADDynamicYield:activateArmedPair(pair, current)
    if pair == nil or pair.mode ~= "DUAL" then
        return dyduOriginalActivateArmedPair(self, pair, current)
    end

    local plan, reason = dyduBuildPlan(self, pair.firstVehicle, pair.secondVehicle, current)
    if plan == nil then
        self:log(
            "Pair %d DUAL failed at trigger: %s; trying SINGLE",
            pair.id,
            tostring(reason)
        )
        local firstVehicle = pair.firstVehicle
        local secondVehicle = pair.secondVehicle
        self:clearPair(pair, "dual trigger unavailable")
        dyduOriginalCreatePair(self, firstVehicle, secondVehicle, current)
        return false
    end

    self:injectApproachCandidate(plan.firstCandidate)
    self:injectApproachCandidate(plan.secondCandidate)
    plan.firstCandidate.meetingRouteIndex = plan.firstCandidate.generatedStartIndex
        + plan.firstCandidate.meetingPointIndex - 1
    plan.secondCandidate.meetingRouteIndex = plan.secondCandidate.generatedStartIndex
        + plan.secondCandidate.meetingPointIndex - 1

    pair.plan = plan
    pair.firstCandidate = plan.firstCandidate
    pair.secondCandidate = plan.secondCandidate
    pair.candidate = plan.firstCandidate
    pair.phase = "DUAL_ACTIVE"
    pair.createdAt = g_time or 0
    pair.activatedAt = g_time or 0
    pair.dualSync = {}

    self.vehiclePairs[pair.firstVehicle] = {
        pair = pair,
        role = "dual",
        otherVehicle = pair.secondVehicle
    }
    self.vehiclePairs[pair.secondVehicle] = {
        pair = pair,
        role = "dual",
        otherVehicle = pair.firstVehicle
    }

    self:log(
        "Pair %d DUAL triggered: route separation=%.1fm world=%.1fm meeting=%.1fm; both move RIGHT",
        pair.id,
        current.routeSeparation,
        current.worldSeparation,
        plan.meetingCoordinate
    )
    self:log(
        "Pair %d dual geometry: %s train=%.1fm offset=%.2fm ramp=%.1fm; %s train=%.1fm offset=%.2fm ramp=%.1fm; gap=%.2fm",
        pair.id,
        dyduLabel(pair.firstVehicle),
        plan.firstCandidate.trainLength,
        plan.firstCandidate.offset,
        plan.firstCandidate.ramp,
        dyduLabel(pair.secondVehicle),
        plan.secondCandidate.trainLength,
        plan.secondCandidate.offset,
        plan.secondCandidate.ramp,
        self.DUAL_TOTAL_GAP
    )
    return true
end

local function dyduGetMeetingDistances(manager, pair)
    if pair == nil or pair.firstCandidate == nil or pair.secondCandidate == nil then
        return nil
    end
    local firstDistance = dyduDistanceToIndex(
        manager,
        pair.firstVehicle,
        pair.firstCandidate.routeTable,
        pair.firstCandidate.meetingRouteIndex
    )
    local secondDistance = dyduDistanceToIndex(
        manager,
        pair.secondVehicle,
        pair.secondCandidate.routeTable,
        pair.secondCandidate.meetingRouteIndex
    )
    if firstDistance == math.huge or secondDistance == math.huge then
        return nil
    end
    return firstDistance, secondDistance
end

local function dyduGetSynchronizedLimit(manager, pair, vehicle)
    local firstDistance, secondDistance = dyduGetMeetingDistances(manager, pair)
    if firstDistance == nil then
        return manager.DUAL_MIN_ROLL_SPEED
    end

    if pair.phase == "DUAL_CLEARING" then
        return manager.DUAL_CLEAR_SPEED
    end

    local baseSpeedMs = manager.DUAL_PASS_SPEED / 3.6
    local firstObservedMs = math.max((pair.firstVehicle.lastSpeedReal or 0) * 1000, 1.2)
    local secondObservedMs = math.max((pair.secondVehicle.lastSpeedReal or 0) * 1000, 1.2)
    firstObservedMs = math.min(firstObservedMs, baseSpeedMs)
    secondObservedMs = math.min(secondObservedMs, baseSpeedMs)

    local targetTime = math.max(
        firstDistance / math.max(firstObservedMs, 0.5),
        secondDistance / math.max(secondObservedMs, 0.5),
        math.max(firstDistance, secondDistance) / math.max(baseSpeedMs, 0.5),
        0.5
    )
    local firstTarget = dyduClamp(
        firstDistance / targetTime * 3.6,
        manager.DUAL_MIN_ROLL_SPEED,
        manager.DUAL_PASS_SPEED
    )
    local secondTarget = dyduClamp(
        secondDistance / targetTime * 3.6,
        manager.DUAL_MIN_ROLL_SPEED,
        manager.DUAL_PASS_SPEED
    )

    pair.dualSync = pair.dualSync or {}
    local firstPrevious = pair.dualSync.firstLimit
    local secondPrevious = pair.dualSync.secondLimit
    pair.dualSync.firstLimit = firstPrevious == nil
        and firstTarget or firstPrevious * 0.75 + firstTarget * 0.25
    pair.dualSync.secondLimit = secondPrevious == nil
        and secondTarget or secondPrevious * 0.75 + secondTarget * 0.25

    local now = g_time or 0
    if pair.dualSync.lastLog == nil or now - pair.dualSync.lastLog >= manager.DUAL_LOG_INTERVAL_MS then
        pair.dualSync.lastLog = now
        manager:log(
            "Pair %d dual timing: %s %.1fm @ %.1fkm/h; %s %.1fm @ %.1fkm/h",
            pair.id,
            dyduLabel(pair.firstVehicle),
            firstDistance,
            pair.dualSync.firstLimit,
            dyduLabel(pair.secondVehicle),
            secondDistance,
            pair.dualSync.secondLimit
        )
    end

    return vehicle == pair.firstVehicle
        and pair.dualSync.firstLimit or pair.dualSync.secondLimit
end

local dyduOriginalShouldHold = ADDynamicYield.shouldHold
function ADDynamicYield:shouldHold(vehicle)
    local state = self:getPairState(vehicle)
    if state ~= nil and state.pair ~= nil and state.pair.mode == "DUAL" then
        local pair = state.pair
        if pair.phase == "ARMED" or pair.phase == "DUAL_CLEARING" then
            return false
        end
        if pair.phase == "DUAL_ACTIVE" then
            local firstDistance, secondDistance = dyduGetMeetingDistances(self, pair)
            if firstDistance == nil then
                return true
            end
            local ownDistance = vehicle == pair.firstVehicle and firstDistance or secondDistance
            local otherDistance = vehicle == pair.firstVehicle and secondDistance or firstDistance
            if ownDistance <= 1 and otherDistance > self.DUAL_EMERGENCY_LEAD then
                if not pair.dualEmergencyLogged then
                    pair.dualEmergencyLogged = true
                    self:log(
                        "Pair %d dual emergency hold: %s reached meeting station %.1fm before partner",
                        pair.id,
                        dyduLabel(vehicle),
                        otherDistance
                    )
                end
                return true
            end
        end
        return false
    end
    return dyduOriginalShouldHold(self, vehicle)
end

local dyduOriginalGetDynamicSpeedLimit = ADDynamicYield.getDynamicSpeedLimit
function ADDynamicYield:getDynamicSpeedLimit(vehicle)
    local state = self:getPairState(vehicle)
    if state ~= nil and state.pair ~= nil and state.pair.mode == "DUAL" then
        if state.pair.phase == "ARMED" then
            return nil
        end
        if state.pair.phase == "DUAL_ACTIVE" or state.pair.phase == "DUAL_CLEARING" then
            return dyduGetSynchronizedLimit(self, state.pair, vehicle)
        end
        return nil
    end
    return dyduOriginalGetDynamicSpeedLimit(self, vehicle)
end

local function dyduProcessPair(manager, pair)
    if pair.cleared then
        return false
    end
    if not manager:isActiveVehicle(pair.firstVehicle)
        or not manager:isActiveVehicle(pair.secondVehicle) then
        manager:clearPair(pair, "dual vehicle inactive")
        return false
    end

    local now = g_time or 0
    if now - (pair.createdAt or now) > manager.DUAL_TIMEOUT_MS then
        manager:clearPair(pair, "dual timeout")
        return false
    end

    if pair.phase == "ARMED" then
        if now - (pair.armedAt or now) > manager.ARMED_TIMEOUT_MS then
            manager:clearPair(pair, "dual armed reservation timeout")
            return false
        end
        if now < (pair.nextDualCheck or 0) then
            return true
        end
        pair.nextDualCheck = now + manager.DUAL_CHECK_INTERVAL_MS

        local current = dyduCurrentOverlap(
            manager,
            pair.firstVehicle,
            pair.secondVehicle,
            pair.overlap
        )
        if current == nil then
            manager:clearPair(pair, "dual armed route changed")
            return false
        end

        if pair.lastDualArmedLog == nil
            or now - pair.lastDualArmedLog >= manager.ARMED_LOG_INTERVAL_MS then
            pair.lastDualArmedLog = now
            manager:log(
                "Pair %d DUAL armed: route separation=%.1fm world=%.1fm trigger=%dm",
                pair.id,
                current.routeSeparation,
                current.worldSeparation,
                manager.DUAL_TRIGGER_DISTANCE
            )
        end

        if current.routeSeparation <= manager.DUAL_TRIGGER_DISTANCE then
            return manager:activateArmedPair(pair, current)
        end
        return true
    end

    if pair.firstCandidate == nil or pair.secondCandidate == nil then
        manager:clearPair(pair, "dual candidates missing")
        return false
    end

    local firstRoute, firstIndex = pair.firstVehicle.ad.drivePathModule:getWayPoints()
    local secondRoute, secondIndex = pair.secondVehicle.ad.drivePathModule:getWayPoints()
    if firstRoute ~= pair.firstCandidate.routeTable or secondRoute ~= pair.secondCandidate.routeTable
        or firstIndex == nil or secondIndex == nil then
        manager:clearPair(pair, "dual route changed")
        return false
    end

    if pair.phase == "DUAL_ACTIVE"
        and firstIndex >= pair.firstCandidate.meetingRouteIndex
        and secondIndex >= pair.secondCandidate.meetingRouteIndex then
        pair.phase = "DUAL_CLEARING"
        manager:log("Pair %d dual meeting completed; both trains clearing and rejoining", pair.id)
    end

    local firstFinished = firstIndex > pair.firstCandidate.generatedEndIndex
    local secondFinished = secondIndex > pair.secondCandidate.generatedEndIndex
    if firstFinished and secondFinished then
        manager:clearPair(pair, "dual passage complete; both vehicles returned to route")
        return false
    end
    return true
end

local dyduOriginalUpdatePairs = ADDynamicYield.updatePairs
function ADDynamicYield:updatePairs()
    local dualPairs = {}
    local otherPairs = {}
    for _, pair in ipairs(self.pairs) do
        if pair.mode == "DUAL" then
            table.insert(dualPairs, pair)
        else
            table.insert(otherPairs, pair)
        end
    end

    self.pairs = otherPairs
    dyduOriginalUpdatePairs(self)

    for _, pair in ipairs(dualPairs) do
        if dyduProcessPair(self, pair) then
            table.insert(self.pairs, pair)
        end
    end
end

local dyduOriginalLoadMap = ADDynamicYield.loadMap
function ADDynamicYield:loadMap(name)
    dyduOriginalLoadMap(self, name)
    self.enabled = true
    self.debugEnabled = true
    self:log(
        "%s active; priority DUAL -> SINGLE -> SERIAL, half-gap=%.2fm each, total gap=%.2fm",
        self.BUILD,
        self.DUAL_HALF_GAP,
        self.DUAL_TOTAL_GAP
    )
end
