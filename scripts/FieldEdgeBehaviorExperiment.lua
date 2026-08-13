-- Experimental staged approach for combine unloaders starting outside a field.
-- The long-range pathfinder may find a formally valid route that repeatedly leaves and
-- re-enters the field. For this experiment we only keep enough of that first route to
-- bring the complete tractor/trailer train onto the field. Once there, the stock
-- CatchCombinePipeTask requests a fresh path with the normal field restriction active.
if ADFieldEdgeDiagnostics == nil or CatchCombinePipeTask == nil or PathFinderModule == nil then
    Logging.error("[AD-FE] behavior experiment prerequisites unavailable")
    return
end

local D = ADFieldEdgeDiagnostics
D.BEHAVIOR_BUILD = "field-edge-exp0.2"
D.ENTRY_TRAIN_MARGIN_M = 0.50
D.STRICT_FIELD_MIN_STEPS = 200

local function getTrainLength(vehicle)
    if AutoDrive.getTractorTrainLength ~= nil then
        local length = AutoDrive.getTractorTrainLength(vehicle, true, false)
        if length ~= nil and length > 0 then
            return length
        end
    end
    if vehicle ~= nil and vehicle.size ~= nil and vehicle.size.length ~= nil then
        return vehicle.size.length
    end
    return 10
end

-- Diagnostics already wrap startPathPlanningToPipe before this file is loaded.
local originalStartPathPlanningToPipe = PathFinderModule.startPathPlanningToPipe
function PathFinderModule:startPathPlanningToPipe(combine, chasing)
    local result = originalStartPathPlanningToPipe(self, combine, chasing)

    self._adfeEntryStage = (self.startIsOnField == false and self.endIsOnField == true)
    self._adfeEntryPathPrepared = false

    if self._adfeEntryStage then
        D:log(
            "ENTRY_STAGE run=%s startOutside=true targetOnField=true action=truncateFirstPathAfterFullTrainEntry",
            tostring(self._adfeRunId or "n/a")
        )
    elseif self.startIsOnField and self.endIsOnField and AutoDrive.getSetting("restrictToField", self.vehicle) then
        -- For a moving combine, stock startPathPlanningToPipe(..., false) permits fallback.
        -- During this experiment a same-field catch must never solve failure by leaving
        -- the field. Mark it as a chase so the pathfinder's fallback modes stay disabled.
        self.chasingVehicle = true
        self._adfeStrictFieldChase = true
        local before = self.max_pathfinder_steps or 0
        self.max_pathfinder_steps = math.max(before, D.STRICT_FIELD_MIN_STEPS)
        D:log(
            "STRICT_FIELD run=%s fallbackDisabled=true maxSteps=%d->%d",
            tostring(self._adfeRunId or "n/a"),
            before,
            self.max_pathfinder_steps
        )
    else
        self._adfeStrictFieldChase = false
    end

    return result
end

local originalGetPath = PathFinderModule.getPath
function PathFinderModule:getPath()
    local wayPoints = originalGetPath(self)

    if not self._adfeEntryStage or self._adfeEntryPathPrepared or wayPoints == nil or #wayPoints < 2 then
        return wayPoints
    end
    self._adfeEntryPathPrepared = true

    local trainLength = getTrainLength(self.vehicle)
    local requiredInsideDistance = trainLength + D.ENTRY_TRAIN_MARGIN_M
    local segmentStart = nil
    local segmentDistance = 0
    local previous = nil
    local cutIndex = nil

    for index, point in ipairs(wayPoints) do
        local pointY = point.y or 0
        local onField = AutoDrive.checkIsOnField(point.x, pointY, point.z)

        if onField then
            if segmentStart == nil then
                segmentStart = index
                segmentDistance = 0
                previous = point
            else
                segmentDistance = segmentDistance + MathUtil.vector2Length(point.x - previous.x, point.z - previous.z)
                previous = point
            end

            if segmentDistance >= requiredInsideDistance then
                cutIndex = index
                break
            end
        else
            segmentStart = nil
            segmentDistance = 0
            previous = nil
        end
    end

    if cutIndex ~= nil and cutIndex < #wayPoints then
        local stagedPath = {}
        for index = 1, cutIndex do
            stagedPath[index] = wayPoints[index]
        end
        D:log(
            "ENTRY_TRUNCATE run=%s originalWayPoints=%d stagedWayPoints=%d firstStableFieldIndex=%d insideDistance=%.1fm trainLength=%.1fm margin=%.2fm",
            tostring(self._adfeRunId or "n/a"),
            #wayPoints,
            #stagedPath,
            segmentStart or -1,
            segmentDistance,
            trainLength,
            D.ENTRY_TRAIN_MARGIN_M
        )
        return stagedPath
    end

    D:log(
        "ENTRY_TRUNCATE_FAILED run=%s wayPoints=%d requiredInsideDistance=%.1fm action=returnOriginalPath",
        tostring(self._adfeRunId or "n/a"),
        #wayPoints,
        requiredInsideDistance
    )
    return wayPoints
end

D:log(
    "%s active; outside-to-field route is truncated after full-train entry, same-field fallback disabled, strict minSteps=%d",
    D.BEHAVIOR_BUILD,
    D.STRICT_FIELD_MIN_STEPS
)
