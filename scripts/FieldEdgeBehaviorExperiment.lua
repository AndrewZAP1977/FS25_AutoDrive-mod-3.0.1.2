-- Safe diagnostic experiment for combine unloaders starting outside their AD work field.
-- No global path is started in this phase. The vehicle stays stopped while we resolve
-- the AD work marker, assigned harvester, exact GIANTS field and nearest boundary point.
if ADFieldEdgeDiagnostics == nil or CatchCombinePipeTask == nil then
    Logging.error("[AD-FE] field-edge-exp0.3 prerequisites unavailable")
    return
end

local D = ADFieldEdgeDiagnostics
D.BEHAVIOR_BUILD = "field-edge-exp0.3"
D._fieldProbeLogged = D._fieldProbeLogged or setmetatable({}, {__mode = "k"})

local function getXZ(vertex)
    if vertex == nil then return nil, nil end
    if vertex.x ~= nil and vertex.z ~= nil then return vertex.x, vertex.z end
    if vertex[1] ~= nil and vertex[2] ~= nil then return vertex[1], vertex[2] end
    return nil, nil
end

local function pointInPolygon(x, z, vertices)
    if vertices == nil or #vertices < 3 then return false end
    local inside = false
    local j = #vertices
    for i = 1, #vertices do
        local xi, zi = getXZ(vertices[i])
        local xj, zj = getXZ(vertices[j])
        if xi ~= nil and zi ~= nil and xj ~= nil and zj ~= nil and ((zi > z) ~= (zj > z)) then
            local denom = zj - zi
            if math.abs(denom) > 0.000001 then
                local crossX = (xj - xi) * (z - zi) / denom + xi
                if x < crossX then inside = not inside end
            end
        end
        j = i
    end
    return inside
end

local function nearestPointOnSegment(px, pz, ax, az, bx, bz)
    local vx, vz = bx - ax, bz - az
    local lenSq = vx * vx + vz * vz
    if lenSq < 0.000001 then
        return ax, az, MathUtil.vector2Length(px - ax, pz - az)
    end
    local t = ((px - ax) * vx + (pz - az) * vz) / lenSq
    t = math.max(0, math.min(1, t))
    local x, z = ax + t * vx, az + t * vz
    return x, z, MathUtil.vector2Length(px - x, pz - z)
end

local function nearestBoundaryPoint(px, pz, vertices)
    if vertices == nil or #vertices < 2 then return nil, nil, nil, nil end
    local bestX, bestZ, bestDistance, bestSegment = nil, nil, math.huge, nil
    for i = 1, #vertices do
        local n = i + 1
        if n > #vertices then n = 1 end
        local ax, az = getXZ(vertices[i])
        local bx, bz = getXZ(vertices[n])
        if ax ~= nil and az ~= nil and bx ~= nil and bz ~= nil then
            local x, z, distance = nearestPointOnSegment(px, pz, ax, az, bx, bz)
            if distance < bestDistance then
                bestX, bestZ, bestDistance, bestSegment = x, z, distance, i
            end
        end
    end
    if bestDistance == math.huge then return nil, nil, nil, nil end
    return bestX, bestZ, bestDistance, bestSegment
end

local function getFieldVertices(field)
    if field == nil or field.getDensityMapPolygon == nil then return nil end
    local polygon = field:getDensityMapPolygon()
    if polygon == nil or polygon.getVerticesList == nil then return nil end
    return polygon:getVerticesList()
end

local function resolveExactField(combineX, combineZ, combineFarmlandId)
    if g_fieldManager == nil or g_fieldManager.getFields == nil then
        return nil, nil, "fieldManagerUnavailable"
    end
    local fallbackField, fallbackVertices = nil, nil
    for _, field in pairs(g_fieldManager:getFields() or {}) do
        local farmlandId = field.farmland ~= nil and field.farmland.id or nil
        if combineFarmlandId == nil or farmlandId == combineFarmlandId then
            local vertices = getFieldVertices(field)
            if fallbackField == nil then
                fallbackField, fallbackVertices = field, vertices
            end
            if vertices ~= nil and pointInPolygon(combineX, combineZ, vertices) then
                return field, vertices, "polygonContainsCombine"
            end
        end
    end
    if fallbackField ~= nil then return fallbackField, fallbackVertices, "farmlandFallback" end
    return nil, nil, "noMatchingField"
end

local function markerName(marker)
    return marker ~= nil and tostring(marker.name or "?") or "nil"
end

local function probeWorkField(task)
    local vehicle, combine = task.vehicle, task.combine
    if vehicle == nil or combine == nil or vehicle.ad == nil or vehicle.ad.stateModule == nil then return end

    local marker = vehicle.ad.stateModule:getFirstMarker()
    local markerWp = marker ~= nil and marker.id ~= nil and ADGraphManager:getWayPointById(marker.id) or nil
    local vx, vy, vz = getWorldTranslation(vehicle.components[1].node)
    local cx, _, cz = getWorldTranslation(combine.components[1].node)

    local vehicleFarmlandId = g_farmlandManager ~= nil and g_farmlandManager:getFarmlandIdAtWorldPosition(vx, vz) or nil
    local combineFarmlandId = g_farmlandManager ~= nil and g_farmlandManager:getFarmlandIdAtWorldPosition(cx, cz) or nil
    local markerFarmlandId = markerWp ~= nil and g_farmlandManager ~= nil and g_farmlandManager:getFarmlandIdAtWorldPosition(markerWp.x, markerWp.z) or nil

    local field, vertices, method = resolveExactField(cx, cz, combineFarmlandId)
    local giantsFieldId = field ~= nil and field.getId ~= nil and field:getId() or nil
    local fieldFarmlandId = field ~= nil and field.farmland ~= nil and field.farmland.id or nil
    local nearestX, nearestZ, nearestDistance, segment = nearestBoundaryPoint(vx, vz, vertices)
    local width = D:getTrainWidth(vehicle)
    local outsideOffset = width * 0.5 + 0.01

    D:log(
        "WORK_GROUP markerName=%s markerIndex=%s waypointId=%s markerPos=%s markerFarmland=%s vehicleFarmland=%s combineFarmland=%s assignedCombine=%s",
        markerName(marker), tostring(marker ~= nil and marker.markerIndex or "nil"), tostring(marker ~= nil and marker.id or "nil"),
        markerWp ~= nil and string.format("(%.1f,%.1f)", markerWp.x, markerWp.z) or "nil",
        tostring(markerFarmlandId), tostring(vehicleFarmlandId), tostring(combineFarmlandId),
        tostring(combine.getName ~= nil and combine:getName() or combine)
    )
    D:log(
        "TARGET_FIELD giantsFieldId=%s fieldFarmland=%s resolve=%s polygonVertices=%s combine=(%.1f,%.1f) combineInsidePolygon=%s",
        tostring(giantsFieldId), tostring(fieldFarmlandId), tostring(method), tostring(vertices ~= nil and #vertices or 0),
        cx, cz, tostring(vertices ~= nil and pointInPolygon(cx, cz, vertices) or false)
    )
    D:log(
        "TARGET_BOUNDARY vehicle=(%.1f,%.1f) nearest=%s distance=%s segment=%s trainWidth=%.2fm outsideOffset=%.2fm",
        vx, vz, nearestX ~= nil and string.format("(%.1f,%.1f)", nearestX, nearestZ) or "nil",
        nearestDistance ~= nil and string.format("%.2fm", nearestDistance) or "nil", tostring(segment), width, outsideOffset
    )
    D:log(
        "FIELD_GATE globalPF=BLOCKED markerMatchesCombineFarmland=%s vehicleOnAnyField=%s action=standStillDiagnosticOnly",
        tostring(markerFarmlandId ~= nil and combineFarmlandId ~= nil and markerFarmlandId == combineFarmlandId),
        tostring(AutoDrive.checkIsOnField(vx, vy, vz))
    )
end

local originalStartNewPathFinding = CatchCombinePipeTask.startNewPathFinding
function CatchCombinePipeTask:startNewPathFinding()
    local vehicle, combine = self.vehicle, self.combine
    if vehicle ~= nil and combine ~= nil then
        local vx, vy, vz = getWorldTranslation(vehicle.components[1].node)
        local cx, cy, cz = getWorldTranslation(combine.components[1].node)
        if not AutoDrive.checkIsOnField(vx, vy, vz) and AutoDrive.checkIsOnField(cx, cy, cz) then
            if not D._fieldProbeLogged[vehicle] then
                D._fieldProbeLogged[vehicle] = true
                probeWorkField(self)
            end
            if self.waitForCheckTimer ~= nil then self.waitForCheckTimer:timer(false) end
            return false
        end
    end
    return originalStartNewPathFinding(self)
end

D:log("%s active; outside-field global pathfinder is blocked, work-field mapping only, vehicle will stand still", D.BEHAVIOR_BUILD)
