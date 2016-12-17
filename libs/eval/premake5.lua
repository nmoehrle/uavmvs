local mve = require "mve"

project "eval"
    kind "StaticLib"
    language "C++"
    toolset "nvcc"

    files {
        "kernels.cu",
        "../cacc/kd_tree.cu",
        "../cacc/bvh_tree.cu",
    }

    mve.use({})
