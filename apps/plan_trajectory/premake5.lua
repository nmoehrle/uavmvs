local mve = require "mve"
project "plan_trajectory"
    kind "ConsoleApp"
    language "C++"
    toolset "nvcc"

    flags { "RelocatableDeviceCode" }
    defines { "_MWAITXINTRIN_H_INCLUDED", "_FORCE_INLINES" }

    files {
        "plan_trajectory.cu",
        "../../libs/cacc/kd_tree.cu",
        "../../libs/cacc/nnsearch.cu",
    }

    mve.use({ "util" })
