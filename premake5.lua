workspace "uavmvs"
    configurations { "debug", "release", "profile" }
    flags { "C++11" }
    local builddir = path.join(_MAIN_SCRIPT_DIR, "build")
    location (builddir)

    defines { "__ROOT__=\"" .. builddir .. "\"" }

    includedirs { "libs" }

    filter { "system:linux", "not toolset:nvcc" }
        linkoptions "-pthread"

    configuration "release"
        targetdir "build/release"
        optimize "On"

    configuration "debug"
        targetdir "build/debug"
        symbols "On"

    os.execute("git submodule init")
    os.execute("git submodule update")

    premake.path = premake.path .. ";" .. path.getabsolute("elibs")

    include("libs/sim")
    include("libs/fmt")

    include("apps/template")
    include("apps/generate_texture")
    include("apps/generate_point_cloud")
    include("apps/simulator")
    include("apps/capture_trajectory")
    include("apps/evaluate_trajectory")
    include("apps/plan_trajectory")
    include("apps/evaluate_reconstruction")
    include("apps/mesh_tools")
    include("apps/create_bundle")
    include("apps/determine_transform")
