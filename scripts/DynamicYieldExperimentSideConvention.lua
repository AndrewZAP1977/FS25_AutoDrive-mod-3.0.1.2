-- Final build marker for the physical right-side convention correction.

if ADDynamicYield == nil then
    Logging.error("[AD-DY] Side-convention marker could not load")
    return
end

ADDynamicYield.BUILD = "dynamic-yield-exp0.10"

local dyscOriginalLoadMap = ADDynamicYield.loadMap
function ADDynamicYield:loadMap(name)
    dyscOriginalLoadMap(self, name)
    self.enabled = true
    self.debugEnabled = true
    self:log(
        "%s physical right-side convention active; right=-X, left pull-off is not allowed",
        self.BUILD
    )
end
