local mve = require "mve"

project "evaluate_trajectory"
    kind "ConsoleApp"
    language "C++"
    toolset "nvcc"

    flags { "RelocatableDeviceCode" }
    defines { "_MWAITXINTRIN_H_INCLUDED", "_FORCE_INLINES" }
    buildoptions { "-Xcompiler -fopenmp" }

    files {
        "evaluate_trajectory.cu",
        "../../libs/eval/kernels.cu",
        "../../libs/cacc/bvh_tree.cu",
        "../../libs/cacc/tracing.cu",
        "../../libs/cacc/nnsearch.cu",
    }

    mve.use({ "util" })

    links { "gomp", "cuda", "cudart" }
