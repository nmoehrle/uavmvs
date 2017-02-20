local m = {}

local filename = ...
local dir = path.getdirectory(path.getabsolute(filename))

newoption {
    trigger = "mve-root",
    value = "path",
    description = "Root directory of MVE"
}

local option = _OPTIONS["mve-root"]

if option and option ~= "" then
    local basedir = path.getabsolute(option);

    if not os.isdir(basedir) then
        print("Wrong path for MVE. Please specify with --mve-root")
        os.exit(-1)
    end

    m.basedir = basedir
else
    for i, v in ipairs({"../../../mve", "../../mve"}) do
        local basedir = path.getabsolute(path.join(dir, v))
        if os.isdir(basedir) then
            m.basedir = basedir
        end
    end
end

if not m.basedir then
    print("Could not find MVE. Please specify with --mve-root")
    os.exit(-1)
end

function m.use(libs)
    includedirs { m.basedir .. "/libs" }

    if libs then
        for _, lib in pairs(libs) do
            if not lib ~= "util" then
                libdirs { m.basedir .. "/libs/" .. lib }
                links { "mve_" .. lib }
            end
        end
    end

    libdirs { m.basedir .. "/libs/mve" }
    links { "mve" }

    if libs then
        if table.contains(libs, "util") then
            libdirs { m.basedir .. "/libs/util" }
            links { "mve_util" }
        end

        if table.contains(libs, "fssr") then
            links { "gomp" }
        end
    end

    links { "jpeg", "png", "tiff" }
end

return m
