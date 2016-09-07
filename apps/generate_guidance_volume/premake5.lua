local mve = require "mve"
project "generate_guidance_volume"
    kind "ConsoleApp"
    language "C++"

    toolset "nvcc"

    flags { "RelocatableDeviceCode" }
    defines { "_MWAITXINTRIN_H_INCLUDED", "_FORCE_INLINES" }

    files {
        "generate_guidance_volume.cu",
        "../../libs/cacc/kd_tree.cu",
        "../../libs/cacc/nnsearch.cu",
        "../../libs/cacc/bvh_tree.cu",
        "../../libs/cacc/tracing.cu",
        "../../libs/eval/kernels.cu",
    }

    mve.use({ "util" })
