local m = {}

m.basedir = path.getabsolute("../../mve")

function m.use(libs)
    includedirs { m.basedir .. "/libs" }
    if libs then
        for _, lib in pairs(libs) do
            libdirs { m.basedir .. "/libs/" .. lib }
            links { "mve_" .. lib }
        end
    end

    libdirs { m.basedir .. "/libs/mve" }
    links { "mve" }

    libdirs { os.findlib("png"), os.findlib("jpeg"), os.findlib("tiff") }
    links { "jpeg", "png", "tiff" }
end

return m
