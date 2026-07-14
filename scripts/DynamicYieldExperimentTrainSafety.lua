-- Keep the opposing vehicle stopped until the complete yielding train has moved
-- into the right-side corridor. Long trailer combinations need a much longer
-- parallel section than the tractor-only prototype used.

if ADDynamicYield == nil then
    Logging.error("[AD-DY] Train-safety patch could not load")
    return
end

ADDynamicYield.BUILD = "dynamic-yield-exp0.4"
ADDynamicYield.TRAIN_PLATEAU_MARGIN = 12
ADDynamicYield.TRAIN_HOLD_MARGIN = 4
ADDynamicYield.MIN_TRAIN_PLATEAU = 22
ADDynamicYield.TRAIN_RETURN_MARGIN = 7

local function dytClamp(value, minimum, maximum)
    return math.max(minimum, math.min(maximum, value))
end

local function dytDistance(x1, z1, x2, z2)
    return MathUtil.vector2Length(x2 - x1, z2 - z1)
end

local function dytSmoothStep(value)
    value = dytClamp(value, 0, 1)
    return value * value * (3 - 2 * value)
end

local function dytLabel(vehicle)
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

-- Replaces the exp0.3 candidate geometry. The tractor reaches full lateral
-- offset after the first ramp, then travels at least one complete train length
-- before it is allowed to stop. This lets the last trailer follow off the main
-- route before the opposing vehicle is released.
function ADDynamicYield:buildApproachCandidate(vehicle, otherVehicle, entryDistance, sharedLength)
    local wayPoints, currentIndex = self:getRoute(vehicle)
    if wayPoints == nil then
        return nil, "no route"
    end

    local x, y, z = self:getPosition(vehicle)
    local width = self:getWidth(vehicle)
    local otherWidth = self:getWidth(otherVehicle)
    local trainLength = self:getLength(vehicle)
    local otherLength = self:getLength(otherVehicle)
    local turnRadius = self:getTurnRadius(vehicle)

    local offset = dytClamp(
        width * 0.5 + otherWidth * 0.5 + self.CLEARANCE_MARGIN,
        self.MIN_OFFSET,
        self.MAX_OFFSET
    )
    local ramp = dytClamp(
        math.max(self.MIN_RAMP, turnRadius * 2.2, trainLength * 0.65),
        self.MIN_RAMP,
        self.MAX_RAMP
    )
    local plateau = math.max(
        self.MIN_TRAIN_PLATEAU,
        trainLength + self.TRAIN_PLATEAU_MARGIN
    )
    local totalLength = ramp * 2 + plateau
    local holdDistance = math.min(
        ramp + trainLength + self.TRAIN_HOLD_MARGIN,
        ramp + plateau - self.TRAIN_RETURN_MARGIN
    )
    local maneuverStartDistance = math.max(0, entryDistance) + self.ENTRY_BUFFER
    local requiredSharedLength = self.ENTRY_BUFFER + totalLength + self.REJOIN_MARGIN

    if requiredSharedLength > sharedLength then
        return nil, string.format(
            "shared road too short for full train (need %.1fm, have %.1fm; train %.1fm)",
            requiredSharedLength,
            sharedLength,
            trainLength
        )
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
            profile = dytSmoothStep(localDistance / ramp)
        elseif localDistance <= ramp + plateau then
            profile = 1
        else
            profile = 1 - dytSmoothStep((localDistance - ramp - plateau) / ramp)
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
            local horizontal = dytDistance(previousPoint.x, previousPoint.z, temporary.x, temporary.z)
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
        + offset * 8 + currentSpeed * 0.8 + trainLength * 0.25
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
        length = trainLength,
        trainLength = trainLength,
        otherLength = otherLength,
        offset = offset,
        ramp = ramp,
        plateau = plateau,
        holdDistance = holdDistance,
        totalLength = totalLength,
        grade = grade,
        score = score
    }, nil
end

-- The stock route-conflict suppression for an active pair is intentional, but
-- the through vehicle must not move until the yielding tractor reaches the
-- train-safe hold point. The yielding vehicle is still controlled by the
-- original shouldHold implementation.
local dytOriginalShouldHold = ADDynamicYield.shouldHold
function ADDynamicYield:shouldHold(vehicle)
    local state = self:getPairState(vehicle)
    if state ~= nil and state.role == "through" and state.pair ~= nil
        and state.pair.phase == "MANEUVER" then
        if not state.pair.throughHoldLogged then
            state.pair.throughHoldLogged = true
            self:log(
                "Pair %d holding through vehicle %s until full yielding train is clear",
                state.pair.id,
                dytLabel(vehicle)
            )
        end
        return true
    end
    return dytOriginalShouldHold(self, vehicle)
end

local dytOriginalCreatePair = ADDynamicYield.createPair
function ADDynamicYield:createPair(firstVehicle, secondVehicle, overlap)
    local created = dytOriginalCreatePair(self, firstVehicle, secondVehicle, overlap)
    if created then
        local pair = self.pairs[#self.pairs]
        local candidate = pair ~= nil and pair.candidate or nil
        if pair ~= nil and candidate ~= nil then
            self:log(
                "Pair %d train-safe geometry: train=%.1fm ramp=%.1fm plateau=%.1fm hold=%.1fm total=%.1fm",
                pair.id,
                candidate.trainLength or candidate.length or 0,
                candidate.ramp or 0,
                candidate.plateau or 0,
                candidate.holdDistance or 0,
                candidate.totalLength or 0
            )
        end
    end
    return created
end

local dytOriginalLoadMap = ADDynamicYield.loadMap
function ADDynamicYield:loadMap(name)
    dytOriginalLoadMap(self, name)
    self.enabled = true
    self.debugEnabled = true
    self:log(
        "%s train safety active; through vehicle held until full train reaches side corridor",
        self.BUILD
    )
end
