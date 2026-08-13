-- Experimental stabilization for long-range combine chasing.
if ADFieldEdgeDiagnostics == nil or CatchCombinePipeTask == nil or PathFinderModule == nil then Logging.error("[AD-FE] behavior experiment prerequisites unavailable") return end
local D=ADFieldEdgeDiagnostics
D.BEHAVIOR_BUILD="field-edge-exp0.1"
D.PROGRESS_MIN_GAIN_M=2.0
D.PROGRESS_FRESH_MS=6000
D.REPLAN_SUPPRESS_DRIFT_M=75
D.REPLAN_NEAR_DISTANCE_M=300
D.STRICT_FIELD_MIN_STEPS=200
local originalStartPathPlanningToPipe=PathFinderModule.startPathPlanningToPipe
function PathFinderModule:startPathPlanningToPipe(combine,chasing)
 local result=originalStartPathPlanningToPipe(self,combine,chasing)
 if self.startIsOnField and self.endIsOnField and AutoDrive.getSetting("restrictToField",self.vehicle) then
  self.chasingVehicle=true
  self._adfeStrictFieldChase=true
  local before=self.max_pathfinder_steps or 0
  self.max_pathfinder_steps=math.max(before,D.STRICT_FIELD_MIN_STEPS)
  D:log("STRICT_FIELD run=%s fallbackDisabled=true maxSteps=%d->%d",tostring(self._adfeRunId or "n/a"),before,self.max_pathfinder_steps)
 else self._adfeStrictFieldChase=false end
 return result
end
local originalCatchCombineUpdate=CatchCombinePipeTask.update
function CatchCombinePipeTask:update(dt)
 if self.state==CatchCombinePipeTask.STATE_DRIVING and self.vehicle~=nil and self.combine~=nil and self.combinesStartLocation~=nil then
  local pf=self.vehicle.ad~=nil and self.vehicle.ad.pathFinderModule or nil
  local runId=pf~=nil and pf._adfeRunId or nil
  local now=D:nowMs()
  local distanceToCombine=AutoDrive.getDistanceBetween(self.vehicle,self.combine)
  if self._adfeProgressRun~=runId then
   self._adfeProgressRun=runId; self._adfeBestCombineDistance=distanceToCombine; self._adfeLastProgressMs=now
  elseif distanceToCombine<(self._adfeBestCombineDistance or math.huge)-D.PROGRESS_MIN_GAIN_M then
   self._adfeBestCombineDistance=distanceToCombine; self._adfeLastProgressMs=now
  end
  local cx,cy,cz=getWorldTranslation(self.combine.components[1].node)
  local drift=MathUtil.vector2Length(cx-self.combinesStartLocation.x,cz-self.combinesStartLocation.z)
  local progressFresh=now-(self._adfeLastProgressMs or 0)<=D.PROGRESS_FRESH_MS
  if drift>=D.REPLAN_SUPPRESS_DRIFT_M and distanceToCombine>D.REPLAN_NEAR_DISTANCE_M and progressFresh then
   self.combinesStartLocation.x=cx; self.combinesStartLocation.y=cy; self.combinesStartLocation.z=cz
   D:log("REPLAN_SUPPRESSED run=%s drift=%.1fm distanceToCombine=%.1fm reason=productivePath",tostring(runId or "n/a"),drift,distanceToCombine)
  end
 end
 return originalCatchCombineUpdate(self,dt)
end
D:log("%s active; productive long-range path kept, same-field fallback disabled, strict minSteps=%d",D.BEHAVIOR_BUILD,D.STRICT_FIELD_MIN_STEPS)
