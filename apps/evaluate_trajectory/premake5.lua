local mve = require "mve"

project "evaluate_trajectory"
    kind "ConsoleApp"
    language "C++"
    toolset "nvcc"

    flags { "RelocatableDeviceCode" }
    defines { "_MWAITXINTRIN_H_INCLUDED", "_FORCE_INLINES" }
    buildoptions = { "-fopenmp" }

    files {
        "evaluate_trajectory.cu",
        "kernel.cu",
        "../../libs/cacc/bvh_tree.cu",
        "../../libs/cacc/tracing.cu"
    }

    mve.use({ "util" })

    links { "gomp", "cuda", "cudart" }
