local mve = require "mve"
project "plan_trajectory"
    kind "ConsoleApp"
    language "C++"
    toolset "nvcc"

    buildoptions { "-Xcompiler -fopenmp" }

    flags { "RelocatableDeviceCode" }
    defines { "_MWAITXINTRIN_H_INCLUDED", "_FORCE_INLINES" }

    files {
        "plan_trajectory.cu",
        "../../libs/cacc/kd_tree.cu",
        "../../libs/cacc/nnsearch.cu",
        "../../libs/cacc/bvh_tree.cu",
        "../../libs/cacc/tracing.cu",
        "../../libs/eval/kernels.cu",
    }

    mve.use({ "util" })

    links { "fmt", "gomp", "utp" }
