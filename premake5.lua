workspace "uavmvs"
    configurations { "debug", "release", "profile" }
    flags { "C++11" }
    warnings "Extra"

    location "build"

    defines { "__ROOT__=\"" .. _MAIN_SCRIPT_DIR .. "\"" }

    includedirs { "libs" }

    filter { "system:linux", "not toolset:nvcc" }
        linkoptions "-pthread"

    filter { "toolset:nvcc" }
        buildoptions {
            "--gpu-architecture sm_52",
            "-Xcompiler -Wno-unknown-pragmas",
            "--ftz=true",
            "--prec-div=false",
            "--prec-sqrt=false",
            "--resource-usage",
        }
        linkoptions { "--gpu-architecture sm_52" }

    configuration "release"
        targetdir "build/release"
        optimize "On"

    configuration "debug"
        targetdir "build/debug"
        symbols "On"

    os.execute("git submodule init")
    os.execute("git submodule update")

    premake.path = premake.path .. ";" .. path.getabsolute("elibs")

    include("libs/fmt")
    include("libs/sim")
    include("libs/utp")
    include("libs/eval")

    include("apps/template")

    include("apps/testing")

    include("apps/mesh_utils")

    include("apps/mesh_tools")
    include("apps/image_tools")
    include("apps/scene_tools")
    include("apps/cloud_tools")
    include("apps/volume_tools")

    include("apps/create_bundle")
    include("apps/determine_matching_mask")

    include("apps/estimate_ground_plane")

    include("apps/generate_texture")
    include("apps/generate_proxy_mesh")
    include("apps/generate_proxy_cloud")
    include("apps/generate_grid_trajectory")
    include("apps/generate_guidance_volume")
    include("apps/generate_initial_trajectory")

    include("apps/selector")
    include("apps/simulator")
    include("apps/visualizer")

    include("apps/plan_trajectory")
    include("apps/capture_trajectory")
    include("apps/optimize_trajectory")
    include("apps/evaluate_trajectory")
    include("apps/interpolate_trajectory")
    include("apps/evaluate_reconstruction")
    include("apps/estimate_capture_difficulty")
