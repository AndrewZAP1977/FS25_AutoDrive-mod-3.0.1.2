if ADFieldEdgeDiagnostics == nil or CatchCombinePipeTask == nil then
    Logging.error("[AD-FE] field-edge-exp0.3.1 prerequisites unavailable")
    return
end

local D = ADFieldEdgeDiagnostics
D.BEHAVIOR_BUILD = "field-edge-exp0.3.1"
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

local function nearestPointOnSegment(px, pz, a, b)
    local dx, dz = b.x - a.x, b.z - a.z
    local len2 = dx * dx + dz * dz
    local t = 0
    if len2 > 0.000001 then
        t = ((px - a.x) * dx + (pz - a.z) * dz) / len2
        t = math.max(0, math.min(1, t))
    end
    local x, z = a.x + t * dx, a.z + t * dz
    return x, z, MathUtil.vector2Length(px - x, pz - z)
end

local function nearestBoundaryPoint(px, pz, vertices)
    if vertices == nil or #vertices < 2 then return nil, nil, nil, nil end
    local bx, bz, bd, bi = nil, nil, math.huge, nil
    for i = 1, #vertices do
        local j = i == #vertices and 1 or i + 1
        local x, z, d = nearestPointOnSegment(px, pz, vertices[i], vertices[j])
        if d < bd then bx, bz, bd, bi = x, z, d, i end
    end
    return bx, bz, bd, bi
end

local function sameMarker(a, b)
    if a == nil or b == nil then return false end
    if a == b then return true end
    return a.markerIndex ~= nil and b.markerIndex ~= nil and a.markerIndex == b.markerIndex
end

local function probeWorkField(task)
    local vehicle, combine = task.vehicle, task.combine
    if vehicle == nil or combine == nil then return end

    local marker = vehicle.ad.stateModule:getFirstMarker()
    local combineMarker = combine.ad ~= nil and combine.ad.stateModule ~= nil and combine.ad.stateModule:getFirstMarker() or nil
    local markerWp = marker ~= nil and marker.id ~= nil and ADGraphManager:getWayPointById(marker.id) or nil

    local vx, vy, vz = getWorldTranslation(vehicle.components[1].node)
    local cx, _, cz = getWorldTranslation(combine.components[1].node)
    local vehicleFarmlandId = g_farmlandManager:getFarmlandIdAtWorldPosition(vx, vz)
    local combineFarmlandId = g_farmlandManager:getFarmlandIdAtWorldPosition(cx, cz)
    local markerFarmlandId = markerWp ~= nil and g_farmlandManager:getFarmlandIdAtWorldPosition(markerWp.x, markerWp.z) or nil

    local farmland = g_farmlandManager:getFarmlandAtWorldPosition(cx, cz)
    local field = farmland ~= nil and farmland.getField ~= nil and farmland:getField() or nil
    local vertices = getFieldVertices(field)
    local fieldId = field ~= nil and field.getId ~= nil and field:getId() or nil
    local nx, nz, nd, segment = nearestBoundaryPoint(vx, vz, vertices)
    local width = D:getMaxTrainWidth(vehicle)

    D:log("WORK_GROUP markerName=%s markerIndex=%s combineMarkerName=%s combineMarkerIndex=%s groupMatch=%s markerFarmland=%s vehicleFarmland=%s combineFarmland=%s assignedCombine=%s",
        tostring(marker ~= nil and marker.name or "nil"), tostring(marker ~= nil and marker.markerIndex or "nil"),
        tostring(combineMarker ~= nil and combineMarker.name or "nil"), tostring(combineMarker ~= nil and combineMarker.markerIndex or "nil"),
        tostring(sameMarker(marker, combineMarker)), tostring(markerFarmlandId), tostring(vehicleFarmlandId), tostring(combineFarmlandId),
        tostring(combine.getName ~= nil and combine:getName() or combine))

    D:log("TARGET_FIELD giantsFieldId=%s polygonVertices=%s combine=(%.1f,%.1f)",
        tostring(fieldId), tostring(vertices ~= nil and #vertices or 0), cx, cz)

    D:log("TARGET_BOUNDARY vehicle=(%.1f,%.1f) nearest=%s distance=%s segment=%s trainWidth=%.2fm outsideOffset=%.2fm",
        vx, vz, nx ~= nil and string.format("(%.1f,%.1f)", nx, nz) or "nil",
        nd ~= nil and string.format("%.2fm", nd) or "nil", tostring(segment), width, width * 0.5 + 0.01)

    D:log("FIELD_GATE globalPF=BLOCKED adWorkGroupMatch=%s vehicleOnAnyField=%s action=standStillDiagnosticOnly",
        tostring(sameMarker(marker, combineMarker)), tostring(AutoDrive.checkIsOnField(vx, vy, vz)))
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

D:log("%s active; global pathfinder blocked outside work field; diagnostic only", D.BEHAVIOR_BUILD)
