if ADFieldEdgeDiagnostics == nil or CatchCombinePipeTask == nil then
    Logging.error("[AD-FE] field-edge-exp0.4 prerequisites unavailable")
    return
end

local D = ADFieldEdgeDiagnostics
D.BEHAVIOR_BUILD = "field-edge-exp0.4"
D.EDGE_TRACE_DISTANCE_M = 45
D.EDGE_POINT_SPACING_M = 3
D.EDGE_TANGENT_WINDOW_M = 5
D.EDGE_LOOKAHEAD_M = 10
D.EDGE_SPEED_KMH = 12
D.EDGE_REACHED_M = 3.5
D.EDGE_SAMPLE_INTERVAL_MS = 1500
D._fieldProbeLogged = D._fieldProbeLogged or setmetatable({}, {__mode = "k"})

local function getFieldVertices(field)
    if field == nil or field.getDensityMapPolygon == nil then return nil end
    local polygon = field:getDensityMapPolygon()
    if polygon == nil or polygon.getVerticesList == nil then return nil end
    local raw = polygon:getVerticesList()
    if raw == nil then return nil end
    local vertices = {}
    for i = 1, #raw, 2 do
        local x, z = raw[i], raw[i + 1]
        if type(x) == "number" and type(z) == "number" then
            vertices[#vertices + 1] = {x = x, z = z}
        end
    end
    return vertices
end

local function pointInPolygon(x, z, vertices)
    if vertices == nil or #vertices < 3 then return false end
    local inside = false
    local j = #vertices
    for i = 1, #vertices do
        local a, b = vertices[i], vertices[j]
        if ((a.z > z) ~= (b.z > z)) then
            local dz = b.z - a.z
            if math.abs(dz) > 0.000001 then
                local crossX = (b.x - a.x) * (z - a.z) / dz + a.x
                if x < crossX then inside = not inside end
            end
        end
        j = i
    end
    return inside
end

local function nearestPointOnSegment(px, pz, a, b)
    local dx, dz = b.x - a.x, b.z - a.z
    local len2 = dx * dx + dz * dz
    local t = 0
    if len2 > 0.000001 then
        t = ((px - a.x) * dx + (pz - a.z) * dz) / len2
        t = math.max(0, math.min(1, t))
    end
    local x, z = a.x + t * dx, a.z + t * dz
    return x, z, MathUtil.vector2Length(px - x, pz - z), t
end

local function nearestBoundaryPoint(px, pz, vertices)
    if vertices == nil or #vertices < 2 then return nil, nil, nil, nil, nil end
    local bx, bz, bd, bi, bt = nil, nil, math.huge, nil, nil
    for i = 1, #vertices do
        local j = i == #vertices and 1 or i + 1
        local x, z, d, t = nearestPointOnSegment(px, pz, vertices[i], vertices[j])
        if d < bd then bx, bz, bd, bi, bt = x, z, d, i, t end
    end
    return bx, bz, bd, bi, bt
end

local function sameMarker(a, b)
    if a == nil or b == nil then return false end
    if a == b then return true end
    return a.markerIndex ~= nil and b.markerIndex ~= nil and a.markerIndex == b.markerIndex
end

local function buildArcTable(vertices)
    local prefix = {}
    local segLen = {}
    local total = 0
    for i = 1, #vertices do
        prefix[i] = total
        local j = i == #vertices and 1 or i + 1
        local len = MathUtil.vector2Length(vertices[j].x - vertices[i].x, vertices[j].z - vertices[i].z)
        segLen[i] = len
        total = total + len
    end
    return prefix, segLen, total
end

local function normalizeArc(s, total)
    if total <= 0 then return 0 end
    s = s % total
    if s < 0 then s = s + total end
    return s
end

local function sampleArc(vertices, prefix, segLen, total, arc)
    arc = normalizeArc(arc, total)
    for i = 1, #vertices do
        local len = segLen[i]
        if arc <= prefix[i] + len or i == #vertices then
            local j = i == #vertices and 1 or i + 1
            local t = len > 0.000001 and math.clamp((arc - prefix[i]) / len, 0, 1) or 0
            return vertices[i].x + (vertices[j].x - vertices[i].x) * t,
                   vertices[i].z + (vertices[j].z - vertices[i].z) * t,
                   i
        end
    end
    return vertices[1].x, vertices[1].z, 1
end

local function polygonSignedArea(vertices)
    local area = 0
    for i = 1, #vertices do
        local j = i == #vertices and 1 or i + 1
        area = area + vertices[i].x * vertices[j].z - vertices[j].x * vertices[i].z
    end
    return area * 0.5
end

local function outsidePointAtArc(vertices, prefix, segLen, total, arc, offset, signedArea)
    local bx, bz = sampleArc(vertices, prefix, segLen, total, arc)
    local ax, az = sampleArc(vertices, prefix, segLen, total, arc - D.EDGE_TANGENT_WINDOW_M)
    local cx, cz = sampleArc(vertices, prefix, segLen, total, arc + D.EDGE_TANGENT_WINDOW_M)
    local tx, tz = cx - ax, cz - az
    local tl = MathUtil.vector2Length(tx, tz)
    if tl < 0.000001 then return nil, nil end
    tx, tz = tx / tl, tz / tl

    -- Positive signed area means CCW: polygon interior is left of the forward edge,
    -- therefore outside is the right normal. Negative area is the opposite.
    local nx, nz
    if signedArea >= 0 then
        nx, nz = tz, -tx
    else
        nx, nz = -tz, tx
    end

    local x, z = bx + nx * offset, bz + nz * offset
    if pointInPolygon(x, z, vertices) then
        -- Defensive fallback for unusual polygon winding/corners.
        x, z = bx - nx * offset, bz - nz * offset
    end
    return x, z
end

local function getWorkField(task)
    local vehicle, combine = task.vehicle, task.combine
    if vehicle == nil or combine == nil or vehicle.ad == nil or vehicle.ad.stateModule == nil then return nil end

    local marker = vehicle.ad.stateModule:getFirstMarker()
    local combineMarker = combine.ad ~= nil and combine.ad.stateModule ~= nil and combine.ad.stateModule:getFirstMarker() or nil
    local cx, _, cz = getWorldTranslation(combine.components[1].node)
    local farmland = g_farmlandManager:getFarmlandAtWorldPosition(cx, cz)
    local field = farmland ~= nil and farmland.getField ~= nil and farmland:getField() or nil
    local vertices = getFieldVertices(field)

    return {
        marker = marker,
        combineMarker = combineMarker,
        groupMatch = sameMarker(marker, combineMarker),
        field = field,
        fieldId = field ~= nil and field.getId ~= nil and field:getId() or nil,
        vertices = vertices
    }
end

local function probeWorkField(task, info)
    local vehicle, combine = task.vehicle, task.combine
    local marker, combineMarker = info.marker, info.combineMarker
    local markerWp = marker ~= nil and marker.id ~= nil and ADGraphManager:getWayPointById(marker.id) or nil
    local vx, vy, vz = getWorldTranslation(vehicle.components[1].node)
    local cx, _, cz = getWorldTranslation(combine.components[1].node)
    local vehicleFarmlandId = g_farmlandManager:getFarmlandIdAtWorldPosition(vx, vz)
    local combineFarmlandId = g_farmlandManager:getFarmlandIdAtWorldPosition(cx, cz)
    local markerFarmlandId = markerWp ~= nil and g_farmlandManager:getFarmlandIdAtWorldPosition(markerWp.x, markerWp.z) or nil
    local nx, nz, nd, segment = nearestBoundaryPoint(vx, vz, info.vertices)
    local width = D:getMaxTrainWidth(vehicle)

    D:log("WORK_GROUP markerName=%s markerIndex=%s combineMarkerName=%s combineMarkerIndex=%s groupMatch=%s markerFarmland=%s vehicleFarmland=%s combineFarmland=%s assignedCombine=%s",
        tostring(marker ~= nil and marker.name or "nil"), tostring(marker ~= nil and marker.markerIndex or "nil"),
        tostring(combineMarker ~= nil and combineMarker.name or "nil"), tostring(combineMarker ~= nil and combineMarker.markerIndex or "nil"),
        tostring(info.groupMatch), tostring(markerFarmlandId), tostring(vehicleFarmlandId), tostring(combineFarmlandId),
        tostring(combine.getName ~= nil and combine:getName() or combine))

    D:log("TARGET_FIELD giantsFieldId=%s polygonVertices=%s combine=(%.1f,%.1f)",
        tostring(info.fieldId), tostring(info.vertices ~= nil and #info.vertices or 0), cx, cz)

    D:log("TARGET_BOUNDARY vehicle=(%.1f,%.1f) nearest=%s distance=%s segment=%s trainWidth=%.2fm outsideOffset=%.2fm",
        vx, vz, nx ~= nil and string.format("(%.1f,%.1f)", nx, nz) or "nil",
        nd ~= nil and string.format("%.2fm", nd) or "nil", tostring(segment), width, width * 0.5 + 0.01)

    D:log("FIELD_GATE globalPF=BLOCKED adWorkGroupMatch=%s vehicleOnAnyField=%s action=localBoundaryTrace",
        tostring(info.groupMatch), tostring(AutoDrive.checkIsOnField(vx, vy, vz)))
end

local function buildBoundaryTrace(task, info)
    local vehicle, combine = task.vehicle, task.combine
    local vertices = info.vertices
    if vertices == nil or #vertices < 3 then return nil, "noPolygon" end

    local vx, _, vz = getWorldTranslation(vehicle.components[1].node)
    local cx, _, cz = getWorldTranslation(combine.components[1].node)
    local _, _, startDist, startSeg, startT = nearestBoundaryPoint(vx, vz, vertices)
    local _, _, _, targetSeg, targetT = nearestBoundaryPoint(cx, cz, vertices)
    if startSeg == nil or targetSeg == nil then return nil, "boundaryProjectionFailed" end

    local prefix, segLen, perimeter = buildArcTable(vertices)
    if perimeter <= 0 then return nil, "zeroPerimeter" end

    local startArc = prefix[startSeg] + (startT or 0) * segLen[startSeg]
    local targetArc = prefix[targetSeg] + (targetT or 0) * segLen[targetSeg]
    local forwardDistance = normalizeArc(targetArc - startArc, perimeter)
    local reverseDistance = perimeter - forwardDistance
    local direction = forwardDistance <= reverseDistance and 1 or -1
    local directionName = direction == 1 and "forward" or "reverse"
    local width = D:getMaxTrainWidth(vehicle)
    local offset = width * 0.5 + 0.01
    local signedArea = polygonSignedArea(vertices)

    local route = {}
    local travelled = 0
    while travelled <= D.EDGE_TRACE_DISTANCE_M + 0.001 do
        local arc = startArc + direction * travelled
        local x, z = outsidePointAtArc(vertices, prefix, segLen, perimeter, arc, offset, signedArea)
        if x == nil then return nil, "outsideOffsetFailed" end
        if pointInPolygon(x, z, vertices) then return nil, "offsetPointInsideField" end
        local y = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, x, 1, z)
        route[#route + 1] = {x = x, y = y, z = z, id = -1, isPathFinderPoint = true}
        travelled = travelled + D.EDGE_POINT_SPACING_M
    end

    if #route < 3 then return nil, "tooFewRoutePoints" end

    return {
        points = route,
        direction = direction,
        directionName = directionName,
        startArc = startArc,
        targetArc = targetArc,
        perimeter = perimeter,
        startSegment = startSeg,
        targetSegment = targetSeg,
        startDistance = startDist,
        forwardDistance = forwardDistance,
        reverseDistance = reverseDistance,
        offset = offset,
        vertices = vertices
    }, nil
end

local function stopVehicle(vehicle)
    vehicle.ad.specialDrivingModule:stopVehicle()
    vehicle.ad.specialDrivingModule:update(0)
end

local function finishTrace(task, reason)
    if task._adfeEdgeStage == "DONE" then return end
    local vehicle = task.vehicle
    task._adfeEdgeStage = "DONE"
    local x, _, z = getWorldTranslation(vehicle.components[1].node)
    local route = task._adfeEdgeTrace ~= nil and task._adfeEdgeTrace.points or nil
    local last = route ~= nil and route[#route] or nil
    local remain = last ~= nil and MathUtil.vector2Length(x - last.x, z - last.z) or -1
    D:log("EDGE_TRACE_STOP reason=%s vehicle=(%.1f,%.1f) remainingToProbeEnd=%.1fm globalPF=BLOCKED",
        tostring(reason), x, z, remain)
    stopVehicle(vehicle)
end

local function updateBoundaryTrace(task, dt)
    local vehicle = task.vehicle
    local trace = task._adfeEdgeTrace
    local route = trace ~= nil and trace.points or nil
    if route == nil or #route < 2 then
        finishTrace(task, "routeMissing")
        return
    end

    local node = vehicle.components[1].node
    if vehicle.getAISteeringNode ~= nil then node = vehicle:getAISteeringNode() end
    local x, y, z = getWorldTranslation(node)

    if pointInPolygon(x, z, trace.vertices) then
        finishTrace(task, "unexpectedEnteredTargetField")
        return
    end

    local last = route[#route]
    if MathUtil.vector2Length(x - last.x, z - last.z) <= D.EDGE_REACHED_M then
        finishTrace(task, "probeComplete")
        return
    end

    local startIndex = math.max(1, task._adfeEdgeRouteIndex or 1)
    local bestIndex, bestDistance = startIndex, math.huge
    for i = startIndex, math.min(#route, startIndex + 8) do
        local d = MathUtil.vector2Length(x - route[i].x, z - route[i].z)
        if d < bestDistance then bestIndex, bestDistance = i, d end
    end
    task._adfeEdgeRouteIndex = bestIndex

    local targetIndex = bestIndex
    local accumulated = MathUtil.vector2Length(x - route[bestIndex].x, z - route[bestIndex].z)
    while targetIndex < #route and accumulated < D.EDGE_LOOKAHEAD_M do
        accumulated = accumulated + MathUtil.vector2Length(route[targetIndex + 1].x - route[targetIndex].x, route[targetIndex + 1].z - route[targetIndex].z)
        targetIndex = targetIndex + 1
    end
    local target = route[targetIndex]

    if vehicle.ad.collisionDetectionModule:hasDetectedObstable(dt) then
        if not task._adfeObstacleLogged then
            task._adfeObstacleLogged = true
            D:log("EDGE_TRACE_OBSTACLE routeIndex=%d/%d vehicle=(%.1f,%.1f) action=stopProbe", bestIndex, #route, x, z)
        end
        finishTrace(task, "obstacleDetected")
        return
    end

    vehicle.ad.specialDrivingModule:releaseVehicle()
    if vehicle.startMotor and not vehicle:getIsMotorStarted() and vehicle:getCanMotorRun() and not vehicle.ad.specialDrivingModule:shouldStopMotor() then
        vehicle:startMotor()
    end
    vehicle.ad.trailerModule:handleTrailerReversing(false)

    local lx, lz = AutoDrive.getDriveDirection(vehicle, target.x, y, target.z, node)
    local maxAngle = 60
    if vehicle.maxRotation then
        maxAngle = vehicle.maxRotation > (2 * math.pi) and vehicle.maxRotation or math.deg(vehicle.maxRotation)
    end
    AutoDrive.driveInDirection(vehicle, dt, maxAngle, 1, 0.8, maxAngle, true, true, lx, lz, D.EDGE_SPEED_KMH, 1)

    local now = D:nowMs()
    if task._adfeNextEdgeSample == nil or now >= task._adfeNextEdgeSample then
        task._adfeNextEdgeSample = now + D.EDGE_SAMPLE_INTERVAL_MS
        local _, _, edgeDistance = nearestBoundaryPoint(x, z, trace.vertices)
        D:log("EDGE_TRACE_SAMPLE routeIndex=%d/%d lookaheadIndex=%d edgeDist=%.2fm speed=%.1fkmh target=(%.1f,%.1f)",
            bestIndex, #route, targetIndex, edgeDistance or -1, (vehicle.lastSpeedReal or 0) * 3600, target.x, target.z)
    end
end

local originalUpdate = CatchCombinePipeTask.update
function CatchCombinePipeTask:update(dt)
    if self._adfeEdgeStage == "TRACE" then
        updateBoundaryTrace(self, dt)
        return
    elseif self._adfeEdgeStage == "DONE" then
        stopVehicle(self.vehicle)
        return
    end

    local vehicle, combine = self.vehicle, self.combine
    if vehicle ~= nil and combine ~= nil then
        local info = getWorkField(self)
        if info ~= nil and info.vertices ~= nil and #info.vertices >= 3 then
            local vx, _, vz = getWorldTranslation(vehicle.components[1].node)
            local insideTargetField = pointInPolygon(vx, vz, info.vertices)
            if not insideTargetField then
                if not D._fieldProbeLogged[vehicle] then
                    D._fieldProbeLogged[vehicle] = true
                    probeWorkField(self, info)
                end

                if not info.groupMatch then
                    D:log("EDGE_TRACE_ABORT reason=ADWorkGroupMismatch globalPF=BLOCKED")
                    self._adfeEdgeStage = "DONE"
                    stopVehicle(vehicle)
                    return
                end

                local trace, err = buildBoundaryTrace(self, info)
                if trace == nil then
                    D:log("EDGE_TRACE_ABORT reason=%s globalPF=BLOCKED", tostring(err))
                    self._adfeEdgeStage = "DONE"
                    stopVehicle(vehicle)
                    return
                end

                self._adfeEdgeTrace = trace
                self._adfeEdgeRouteIndex = 1
                self._adfeEdgeStage = "TRACE"
                D:log("EDGE_TRACE_START fieldId=%s direction=%s startSegment=%d targetSegment=%d startEdgeDistance=%.2fm routePoints=%d traceDistance=%dm offset=%.2fm speed=%dkmh perimeterForward=%.1fm perimeterReverse=%.1fm globalPF=BLOCKED",
                    tostring(info.fieldId), trace.directionName, trace.startSegment, trace.targetSegment, trace.startDistance or -1,
                    #trace.points, D.EDGE_TRACE_DISTANCE_M, trace.offset, D.EDGE_SPEED_KMH, trace.forwardDistance, trace.reverseDistance)
                updateBoundaryTrace(self, dt)
                return
            end
        elseif vehicle ~= nil then
            if not self._adfeNoFieldLogged then
                self._adfeNoFieldLogged = true
                D:log("EDGE_TRACE_ABORT reason=targetFieldUnavailable globalPF=BLOCKED")
            end
            stopVehicle(vehicle)
            return
        end
    end

    return originalUpdate(self, dt)
end

D:log("%s active; exact field polygon, 45m outside-edge motion probe at 12km/h; no global pathfinder, no field entry yet", D.BEHAVIOR_BUILD)
