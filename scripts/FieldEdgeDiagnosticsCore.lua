-- Lightweight field-edge diagnostics core. Behavior unchanged; no 3D drawing.
if ADFieldEdgeDiagnostics ~= nil then return end
if AutoDrive == nil then Logging.error("[AD-FE] AutoDrive unavailable") return end
ADFieldEdgeDiagnostics = {BUILD="field-edge-diag0.1", SAMPLE_INTERVAL_MS=1500, EDGE_SCAN_RADIUS=18, EDGE_SCAN_STEP=1.5, EDGE_SCAN_RAYS=12, nextRunId=1, taskSamples=setmetatable({}, {__mode="k"})}
local function feDistance(x1,z1,x2,z2) return MathUtil.vector2Length(x2-x1,z2-z1) end
ADFieldEdgeDiagnostics.distance2D = feDistance
function ADFieldEdgeDiagnostics:nowMs() return g_time or (g_currentMission ~= nil and g_currentMission.time) or 0 end
function ADFieldEdgeDiagnostics:log(message, ...) Logging.info("[AD-FE] "..tostring(message), ...) end
function ADFieldEdgeDiagnostics:label(vehicle)
 if vehicle==nil then return "none" end
 local name=nil
 if vehicle.ad~=nil and vehicle.ad.stateModule~=nil and vehicle.ad.stateModule.getName~=nil then name=vehicle.ad.stateModule:getName() end
 if (name==nil or name=="") and vehicle.getName~=nil then name=vehicle:getName() end
 return string.format("%s#%s", tostring(name or "vehicle"), tostring(vehicle.id or -1))
end
function ADFieldEdgeDiagnostics:getPosition(vehicle)
 if vehicle==nil or vehicle.components==nil or vehicle.components[1]==nil then return nil,nil,nil end
 return getWorldTranslation(vehicle.components[1].node)
end
function ADFieldEdgeDiagnostics:getMaxTrainWidth(vehicle)
 local width=vehicle~=nil and vehicle.size~=nil and vehicle.size.width or 0
 if AutoDrive.getAllImplements~=nil and vehicle~=nil then
  for _,implement in ipairs(AutoDrive.getAllImplements(vehicle,true) or {}) do
   if implement~=nil and implement.size~=nil then width=math.max(width, implement.size.width or 0) end
  end
 end
 return width
end
function ADFieldEdgeDiagnostics:findNearestFieldEdge(vehicle)
 local x,y,z=self:getPosition(vehicle); if x==nil then return nil,false end
 local startOnField=AutoDrive.checkIsOnField(x,y,z); local wantedState=not startOnField; local best=nil
 for ray=0,self.EDGE_SCAN_RAYS-1 do
  local a=2*math.pi*ray/self.EDGE_SCAN_RAYS; local dx,dz=math.cos(a),math.sin(a); local d=self.EDGE_SCAN_STEP
  while d<=self.EDGE_SCAN_RADIUS do
   if AutoDrive.checkIsOnField(x+dx*d,y,z+dz*d)==wantedState then if best==nil or d<best then best=d end break end
   d=d+self.EDGE_SCAN_STEP
  end
 end
 return best,startOnField
end
