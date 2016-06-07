workspace "uavmvs"
    configurations { "debug", "release", "profile" }
    flags { "C++11" }
    location "build"

    configuration "release"
        targetdir "build/release"
        optimize "On"

    configuration "debug"
        targetdir "build/debug"
        defines { "Symbols" }

    filter { "system:linux" }
        linkoptions "-pthread"

    os.execute("git submodule init")
    os.execute("git submodule update")

    premake.path = premake.path .. ";" .. path.getabsolute("elibs")

    includedirs { "libs" }

    include("apps/generate_texture")
    include("apps/prepare_mesh")
    include("apps/generate_point_cloud")
    include("apps/simulator")
