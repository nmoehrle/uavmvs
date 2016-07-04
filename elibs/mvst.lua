local m = {}

m.mve = require "mve"
m.basedir = path.getabsolute("../../mvs-texturing")

function m.use( libs )
    includedirs { m.basedir .. "/libs" }
    if libs then
        for _, lib in pairs(libs) do
            libdirs { m.basedir .. "/build/libs/" .. lib }
            links { lib }
        end
    end

    m.mve.use({ "util" })
end

return m
