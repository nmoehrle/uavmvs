local mve = require "mve"
project "estimate_capture_difficulty"
    kind "ConsoleApp"
    language "C++"
    toolset "nvcc"

    flags { "RelocatableDeviceCode" }
    defines { "_MWAITXINTRIN_H_INCLUDED", "_FORCE_INLINES" }


    files {
        "estimate_capture_difficulty.cu",
        "../../libs/cacc/kd_tree.cu",
        "../../libs/cacc/nnsearch.cu",
        "../../libs/cacc/bvh_tree.cu",
        "../../libs/cacc/tracing.cu",
        "../../libs/eval/kernels.cu",
    }

    mve.use({ "util" })
