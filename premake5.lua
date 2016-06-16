workspace "uavmvs"
    configurations { "debug", "release", "profile" }
    flags { "C++11" }
    location "build"

    includedirs { "libs" }

    filter { "system:linux", "not toolset:nvcc" }
        linkoptions "-pthread"

    configuration "release"
        targetdir "build/release"
        optimize "On"

    configuration "debug"
        targetdir "build/debug"
        flags { "Symbols" }

    os.execute("git submodule init")
    os.execute("git submodule update")

    premake.path = premake.path .. ";" .. path.getabsolute("elibs")

    include("libs/sim")

    include("apps/generate_texture")
    include("apps/generate_point_cloud")
    include("apps/capture_trajectory")
    --include("apps/simulator")
    include("apps/evaluate_trajectory")
    include("apps/mesh_tools")
