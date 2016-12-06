local mve = require "mve"

project "eval"
    kind "StaticLib"
    language "C++"
    toolset "nvcc"

    flags { "RelocatableDeviceCode" }

    files {
        "kernels.cu",
        "../cacc/kd_tree.cu",
        "../cacc/bvh_tree.cu",
        "../cacc/nnsearch.cu",
    }

    mve.use({})
