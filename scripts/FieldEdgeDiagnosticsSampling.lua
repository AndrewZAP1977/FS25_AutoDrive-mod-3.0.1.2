-- Pathfinder-finish and active-task sampling hooks for AD field-edge diagnostics.
if ADFieldEdgeDiagnostics==nil or PathFinderModule==nil or CatchCombinePipeTask==nil then Logging.error("[AD-FE] sampling prerequisites unavailable") return end
local D=ADFieldEdgeDiagnostics
local originalPathFinderUpdate=PathFinderModule.update
function PathFinderModule:update(dt)
 local result=originalPathFinderUpdate(self,dt)
 if self._adfeRunId~=nil and not self._adfeFinishLogged and self.isFinished and self.smoothDone==true then
  self._adfeFinishLogged=true
  local cx,_,cz=D:getPosition(self._adfeCombine); local drift=(cx~=nil and self._adfeCombineStartX~=nil) and D.distance2D(self._adfeCombineStartX,self._adfeCombineStartZ,cx,cz) or nil
  local gridNodes=0; for _ in pairs(self.grid or {}) do gridNodes=gridNodes+1 end
  D:log("PF_FINISH run=%d elapsed=%dms steps=%s retries=%s fallback=%s/%s/%s gridNodes=%d wayPoints=%d combineDrift=%s",
   self._adfeRunId,math.max(D:nowMs()-(self._adfeStartMs or D:nowMs()),0),tostring(self.steps),tostring(self.retryCounter),
   tostring(self.fallBackMode1),tostring(self.fallBackMode2),tostring(self.fallBackMode3),gridNodes,self.wayPoints~=nil and #self.wayPoints or 0,
   drift~=nil and string.format("%.1fm",drift) or "n/a")
 end
 return result
end
local originalCatchCombineUpdate=CatchCombinePipeTask.update
function CatchCombinePipeTask:update(dt)
 local result=originalCatchCombineUpdate(self,dt)
 if self.vehicle~=nil and self.combine~=nil then
  local now=D:nowMs(); local last=D.taskSamples[self] or -math.huge
  if now-last>=D.SAMPLE_INTERVAL_MS then
   D.taskSamples[self]=now
   local vx,_,vz=D:getPosition(self.vehicle); local cx,_,cz=D:getPosition(self.combine); local edgeDistance,onField=D:findNearestFieldEdge(self.vehicle)
   local pf=self.vehicle.ad~=nil and self.vehicle.ad.pathFinderModule or nil
   local drift=(pf~=nil and cx~=nil and pf._adfeCombineStartX~=nil) and D.distance2D(pf._adfeCombineStartX,pf._adfeCombineStartZ,cx,cz) or nil
   D:log("SAMPLE state=%s onField=%s edgeDist=%s distanceToCombine=%.1fm speed=%.1fkmh pfRun=%s pfBusy=%s combineDriftFromPFStart=%s",
    self.getStateName~=nil and tostring(self:getStateName()) or "unknown",tostring(onField),edgeDistance~=nil and string.format("%.1fm",edgeDistance) or "n/a",
    (vx~=nil and cx~=nil) and D.distance2D(vx,vz,cx,cz) or -1,(self.vehicle.lastSpeedReal or 0)*3600,
    pf~=nil and tostring(pf._adfeRunId) or "n/a",pf~=nil and tostring(not (pf.isFinished and pf.smoothDone==true)) or "n/a",drift~=nil and string.format("%.1fm",drift) or "n/a")
  end
 end
 return result
end
D:log("%s active; behavior unchanged, no 3D debug drawing",D.BUILD)
