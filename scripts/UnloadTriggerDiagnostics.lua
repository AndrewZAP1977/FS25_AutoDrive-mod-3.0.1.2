ADUnloadTriggerDiagnostics = {}

ADUnloadTriggerDiagnostics.UPDATE_INTERVAL = 250
ADUnloadTriggerDiagnostics.SNAPSHOT_INTERVAL = 500
ADUnloadTriggerDiagnostics.ACTIVE_DISTANCE = 120

local function adutGetName(object)
    if object ~= nil and object.getName ~= nil then
        return tostring(object:getName())
    end
    return tostring(object)
end

local function adutBool(value)
    return value and "true" or "false"
end

local function adutGetUnloadMarker(vehicle)
    if vehicle == nil or vehicle.ad == nil or vehicle.ad.stateModule == nil then
        return nil, nil
    end

    local stateModule = vehicle.ad.stateModule
    local marker = nil

    if stateModule:getMode() == AutoDrive.MODE_DELIVERTO then
        marker = stateModule:getFirstMarker()
    elseif stateModule:getMode() ~= AutoDrive.MODE_LOAD then
        marker = stateModule:getSecondMarker()
    end

    if marker == nil or marker.id == nil then
        return nil, nil
    end

    return marker, ADGraphManager:getWayPointById(marker.id)
end

local function adutDistanceToPoint(object, point)
    if object == nil or point == nil or object.components == nil or object.components[1] == nil then
        return math.huge
    end

    local x, _, z = getWorldTranslation(object.components[1].node)
    return MathUtil.vector2Length(x - point.x, z - point.z)
end

local function adutNodeDistanceToPoint(node, point)
    if node == nil or point == nil or not entityExists(node) then
        return math.huge
    end

    local x, _, z = getWorldTranslation(node)
    return MathUtil.vector2Length(x - point.x, z - point.z)
end

local function adutPosition(object)
    if object == nil or object.components == nil or object.components[1] == nil then
        return 0, 0
    end
    local x, _, z = getWorldTranslation(object.components[1].node)
    return x, z
end

local function adutNodePosition(node)
    if node == nil or not entityExists(node) then
        return 0, 0
    end
    local x, _, z = getWorldTranslation(node)
    return x, z
end

function ADUnloadTriggerDiagnostics:loadMap()
    self.elapsed = 0
    self.snapshotElapsed = 0
    self.vehicleStates = setmetatable({}, {__mode = "k"})
    self.trailerStates = setmetatable({}, {__mode = "k"})
    Logging.info("[AD-UT] unload-trigger diagnostics active; behavior unchanged")
end

function ADUnloadTriggerDiagnostics:deleteMap()
    self.vehicleStates = nil
    self.trailerStates = nil
end

function ADUnloadTriggerDiagnostics:getVehiclesToInspect()
    local result = {}
    local seen = {}

    local function addVehicle(vehicle)
        if vehicle ~= nil and vehicle.ad ~= nil and vehicle.ad.stateModule ~= nil and not seen[vehicle] then
            local mode = vehicle.ad.stateModule:getCurrentMode()
            if mode ~= nil and mode.shouldUnloadAtTrigger ~= nil and mode:shouldUnloadAtTrigger() then
                seen[vehicle] = true
                table.insert(result, vehicle)
            end
        end
    end

    if AutoDrive.getControlledVehicle ~= nil then
        addVehicle(AutoDrive.getControlledVehicle())
    end

    if g_currentMission ~= nil and g_currentMission.vehicles ~= nil then
        for _, vehicle in pairs(g_currentMission.vehicles) do
            if vehicle ~= nil and vehicle.ad ~= nil and vehicle.ad.stateModule ~= nil and vehicle.ad.stateModule:isActive() then
                addVehicle(vehicle)
            end
        end
    end

    return result
end

function ADUnloadTriggerDiagnostics:inspectVehicle(vehicle, takeSnapshot)
    local marker, waypoint = adutGetUnloadMarker(vehicle)
    if marker == nil or waypoint == nil then
        return
    end

    local rootDistance = AutoDrive.getDistanceToUnloadPosition(vehicle)
    if rootDistance == math.huge then
        return
    end

    local maxTriggerDistance = AutoDrive.getMaxTriggerDistance(vehicle)
    local bunkerExtra = ADTriggerManager.getMaxBunkerSiloLength()
    local rangeGate = AutoDrive.isInRangeToLoadUnloadTarget(vehicle)
    local updateGate = rangeGate or rootDistance < (bunkerExtra + maxTriggerDistance)
    local isActive = vehicle.ad.stateModule:isActive()
    local targetReached = false

    if isActive and vehicle.ad.drivePathModule ~= nil and vehicle.ad.drivePathModule.isTargetReached ~= nil then
        targetReached = vehicle.ad.drivePathModule:isTargetReached()
    end

    local state = self.vehicleStates[vehicle]
    if state == nil then
        state = {}
        self.vehicleStates[vehicle] = state
    end

    local nearDestination = rootDistance <= self.ACTIVE_DISTANCE
    if not nearDestination and not updateGate and not state.wasNear then
        return
    end
    state.wasNear = nearDestination

    if state.updateGate ~= updateGate then
        Logging.info("[AD-UT] UPDATE_GATE %s vehicle=\"%s\" active=%s mode=%s rootDist=%.2fm maxTrigger=%.2fm bunkerExtra=%.2fm",
            updateGate and "OPEN" or "CLOSED", adutGetName(vehicle), adutBool(isActive), tostring(vehicle.ad.stateModule:getMode()), rootDistance, maxTriggerDistance, bunkerExtra)
        state.updateGate = updateGate
    end

    if state.rangeGate ~= rangeGate then
        Logging.info("[AD-UT] ROOT_RANGE_GATE %s vehicle=\"%s\" rootDist=%.2fm maxTrigger=%.2fm",
            rangeGate and "OPEN" or "CLOSED", adutGetName(vehicle), rootDistance, maxTriggerDistance)
        state.rangeGate = rangeGate
    end

    if state.targetReached ~= targetReached then
        Logging.info("[AD-UT] DESTINATION_REACHED %s vehicle=\"%s\" rootDist=%.2fm",
            targetReached and "YES" or "NO", adutGetName(vehicle), rootDistance)
        state.targetReached = targetReached
    end

    local units = AutoDrive.getAllUnits(vehicle)
    if units == nil then
        return
    end

    for _, trailer in ipairs(units) do
        if trailer ~= nil and trailer.getCurrentDischargeNode ~= nil then
            local dischargeNode = trailer:getCurrentDischargeNode()
            if dischargeNode ~= nil and dischargeNode.node ~= nil then
                local trailerDistance = adutDistanceToPoint(trailer, waypoint)
                local nodeDistance = adutNodeDistanceToPoint(dischargeNode.node, waypoint)
                local trailerGate = trailerDistance <= maxTriggerDistance
                local canDischarge = false
                if trailer.getCanDischargeToObject ~= nil then
                    canDischarge = trailer:getCanDischargeToObject(dischargeNode)
                end

                local dischargeState = -1
                local unloading = false
                if trailer.getDischargeState ~= nil then
                    dischargeState = trailer:getDischargeState()
                    unloading = dischargeState ~= Dischargeable.DISCHARGE_STATE_OFF
                end
                if vehicle.ad.trailerModule ~= nil and vehicle.ad.trailerModule.isUnloadingWithTrailer == trailer then
                    unloading = unloading or vehicle.ad.trailerModule.isUnloading == true
                end

                local trailerState = self.trailerStates[trailer]
                if trailerState == nil then
                    trailerState = {}
                    self.trailerStates[trailer] = trailerState
                end

                if trailerState.trailerGate ~= trailerGate then
                    Logging.info("[AD-UT] TRAILER_RANGE_GATE %s vehicle=\"%s\" trailer=\"%s\" trailerDist=%.2fm maxTrigger=%.2fm",
                        trailerGate and "OPEN" or "CLOSED", adutGetName(vehicle), adutGetName(trailer), trailerDistance, maxTriggerDistance)
                    trailerState.trailerGate = trailerGate
                end

                if trailerState.canDischarge ~= canDischarge then
                    local rootX, rootZ = adutPosition(vehicle)
                    local trailerX, trailerZ = adutPosition(trailer)
                    local nodeX, nodeZ = adutNodePosition(dischargeNode.node)
                    Logging.info("[AD-UT] DISCHARGE_TARGET %s vehicle=\"%s\" trailer=\"%s\" rootDist=%.2fm trailerDist=%.2fm nodeDist=%.2fm updateGate=%s rangeGate=%s trailerGate=%s trigger=%s dischargeObject=%s root=(%.2f,%.2f) trailer=(%.2f,%.2f) node=(%.2f,%.2f) marker=(%.2f,%.2f)",
                        canDischarge and "ENTER" or "EXIT", adutGetName(vehicle), adutGetName(trailer), rootDistance, trailerDistance, nodeDistance,
                        adutBool(updateGate), adutBool(rangeGate), adutBool(trailerGate), adutBool(dischargeNode.trigger ~= nil), adutBool(dischargeNode.dischargeObject ~= nil),
                        rootX, rootZ, trailerX, trailerZ, nodeX, nodeZ, waypoint.x, waypoint.z)
                    trailerState.canDischarge = canDischarge
                end

                if trailerState.unloading ~= unloading then
                    Logging.info("[AD-UT] UNLOADING %s vehicle=\"%s\" trailer=\"%s\" rootDist=%.2fm trailerDist=%.2fm nodeDist=%.2fm dischargeState=%s",
                        unloading and "START" or "STOP", adutGetName(vehicle), adutGetName(trailer), rootDistance, trailerDistance, nodeDistance, tostring(dischargeState))
                    trailerState.unloading = unloading
                end

                if takeSnapshot and (nearDestination or canDischarge or unloading) then
                    Logging.info("[AD-UT] SNAP vehicle=\"%s\" trailer=\"%s\" active=%s mode=%s rootDist=%.2fm trailerDist=%.2fm nodeDist=%.2fm maxTrigger=%.2fm updateGate=%s rangeGate=%s trailerGate=%s canDischarge=%s trigger=%s dischargeObject=%s dischargeState=%s targetReached=%s speed=%.2fkm/h",
                        adutGetName(vehicle), adutGetName(trailer), adutBool(isActive), tostring(vehicle.ad.stateModule:getMode()), rootDistance, trailerDistance, nodeDistance,
                        maxTriggerDistance, adutBool(updateGate), adutBool(rangeGate), adutBool(trailerGate), adutBool(canDischarge), adutBool(dischargeNode.trigger ~= nil),
                        adutBool(dischargeNode.dischargeObject ~= nil), tostring(dischargeState), adutBool(targetReached), (vehicle.lastSpeedReal or 0) * 3600)
                end
            end
        end
    end
end

function ADUnloadTriggerDiagnostics:update(dt)
    if g_server == nil or g_currentMission == nil then
        return
    end

    self.elapsed = (self.elapsed or 0) + dt
    self.snapshotElapsed = (self.snapshotElapsed or 0) + dt

    if self.elapsed < self.UPDATE_INTERVAL then
        return
    end
    self.elapsed = 0

    local takeSnapshot = false
    if self.snapshotElapsed >= self.SNAPSHOT_INTERVAL then
        takeSnapshot = true
        self.snapshotElapsed = 0
    end

    for _, vehicle in ipairs(self:getVehiclesToInspect()) do
        self:inspectVehicle(vehicle, takeSnapshot)
    end
end

function ADUnloadTriggerDiagnostics:draw()
end

function ADUnloadTriggerDiagnostics:keyEvent()
end

function ADUnloadTriggerDiagnostics:mouseEvent()
end

addModEventListener(ADUnloadTriggerDiagnostics)
