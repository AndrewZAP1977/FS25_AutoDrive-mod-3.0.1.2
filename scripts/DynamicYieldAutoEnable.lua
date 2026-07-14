-- Test-build convenience: enable the dynamic-yield experiment automatically.

if ADDynamicYield == nil then
    Logging.error("[AD-DY] Auto-enable patch could not load")
    return
end

ADDynamicYield.BUILD = "dynamic-yield-exp0.2"
ADDynamicYield.enabled = true

local dyAutoEnableOriginalLoadMap = ADDynamicYield.loadMap
function ADDynamicYield:loadMap(name)
    dyAutoEnableOriginalLoadMap(self, name)
    self.enabled = true
    self:log("%s active automatically for this test build; disable with: adDynamicYield off", self.BUILD)
end
