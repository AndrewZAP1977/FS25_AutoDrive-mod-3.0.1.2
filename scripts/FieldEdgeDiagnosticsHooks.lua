-- Assignment and path-start hooks for AD field-edge diagnostics.
if ADFieldEdgeDiagnostics==nil or CombineUnloaderMode==nil or PathFinderModule==nil then Logging.error("[AD-FE] hook prerequisites unavailable") return end
local D=ADFieldEdgeDiagnostics
local originalAssignToHarvester=CombineUnloaderMode.assignToHarvester
function CombineUnloaderMode:assignToHarvester(harvester)
 local shouldLog=self.state==self.STATE_WAIT_TO_BE_CALLED and harvester~=nil
 if shouldLog then
  local vx,_,vz=D:getPosition(self.vehicle); local hx,_,hz=D:getPosition(harvester)
  local edgeDistance,onField=D:findNearestFieldEdge(self.vehicle); local width=D:getMaxTrainWidth(self.vehicle)
  D:log("CALL unloader=%s combine=%s distance=%.1fm onField=%s edgeDist=%s trainWidth=%.2fm desiredOutsideOffset=%.2fm restrictSetting=%s avoidFruit=%s",
   D:label(self.vehicle),D:label(harvester),(vx~=nil and hx~=nil) and D.distance2D(vx,vz,hx,hz) or -1,tostring(onField),
   edgeDistance~=nil and string.format("%.1fm",edgeDistance) or "n/a",width,width/2+0.01,
   tostring(AutoDrive.getSetting("restrictToField",self.vehicle)),tostring(AutoDrive.getSetting("avoidFruit",self.vehicle)))
 end
 local result=originalAssignToHarvester(self,harvester)
 if shouldLog then D:log("ASSIGNED unloader=%s combine=%s modeState=%s",D:label(self.vehicle),D:label(harvester),self.getStateName~=nil and tostring(self:getStateName()) or "unknown") end
 return result
end
local originalStartPathPlanningToPipe=PathFinderModule.startPathPlanningToPipe
function PathFinderModule:startPathPlanningToPipe(combine,chasing)
 local runId=D.nextRunId; D.nextRunId=runId+1; self._adfeRunId=runId; self._adfeStartMs=D:nowMs(); self._adfeFinishLogged=false; self._adfeCombine=combine
 self._adfeCombineStartX,_,self._adfeCombineStartZ=D:getPosition(combine)
 local result=originalStartPathPlanningToPipe(self,combine,chasing)
 local vx,_,vz=D:getPosition(self.vehicle); local td=(vx~=nil and self.target~=nil) and D.distance2D(vx,vz,self.target.x,self.target.z) or nil
 D:log("PF_START run=%d startOnField=%s endOnField=%s restrictSetting=%s effectiveRestrict=%s avoidFruit=%s turnRadius=%.2fm targetDistance=%s targetCell=(%s,%s) maxSteps=%s",
  runId,tostring(self.startIsOnField),tostring(self.endIsOnField),tostring(AutoDrive.getSetting("restrictToField",self.vehicle)),tostring(self.restrictToField),tostring(self.avoidFruitSetting),
  tonumber(self.minTurnRadius) or -1,td~=nil and string.format("%.1fm",td) or "n/a",self.targetCell~=nil and tostring(self.targetCell.x) or "n/a",self.targetCell~=nil and tostring(self.targetCell.z) or "n/a",tostring(self.max_pathfinder_steps))
 return result
end
