local m = {}

local filename = ...
local dir = path.getdirectory(path.getabsolute(filename))

newoption {
    trigger = "mvst-root",
    value = "path",
    description = "Root directory of MVS-Texturing"
}

local option = _OPTIONS["mvst-root"]
if option and option ~= "" then
    local basedir = path.getabsolute(option);

    if not os.isdir(basedir) then
        print("Wrong path for MVS-Texturing. Please specify with --mvst-root")
        os.exit(-1)
    end

    m.basedir = basedir
else
    for i, v in ipairs({"../../../mvs-texturing", "../../mvs-texturing"}) do
        local basedir = path.getabsolute(path.join(dir, v))
        if os.isdir(basedir) then
            m.basedir = basedir
        end
    end
end

if not m.basedir then
    print("Could not find MVS-Texturing. Please specify with --mvst-root")
    os.exit(-1)
end

m.mve = require "mve"

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
