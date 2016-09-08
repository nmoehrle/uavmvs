workspace "uavmvs"
    configurations { "debug", "release", "profile" }
    flags { "C++11" }
    location "build"

    defines { "__ROOT__=\"" .. _MAIN_SCRIPT_DIR .. "\"" }

    includedirs { "libs" }

    filter { "system:linux", "not toolset:nvcc" }
        linkoptions "-pthread"

    filter { "toolset:nvcc" }
        buildoptions {"--gpu-architecture compute_35"}
        linkoptions "--gpu-architecture compute_35"

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

    include("apps/mesh_tools")
    include("apps/image_tools")

    include("apps/create_bundle")
    include("apps/determine_transform")
    include("apps/determine_matching_mask")

    include("apps/estimate_ground_plane")

    include("apps/generate_sphere")
    include("apps/generate_texture")
    include("apps/generate_proxy_mesh")
    include("apps/generate_proxy_cloud")
    include("apps/generate_guidance_volume")

    include("apps/simulator")
    include("apps/visualizer")

    include("apps/plan_trajectory")
    include("apps/capture_trajectory")
    include("apps/evaluate_trajectory")
    include("apps/evaluate_reconstruction")
    include("apps/estimate_capture_difficulty")
