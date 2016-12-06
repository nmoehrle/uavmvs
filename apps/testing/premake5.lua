local mve = require "mve"

project "interop"
    kind "ConsoleApp"
    toolset "nvcc"
    language "C++"

    buildoptions { "-Xcompiler -fopenmp" }

    files {
        "interop.cu",
        "../../libs/cacc/bvh_tree.cu",
    }

    mve.use({ "util", "ogl" })

    links { "gomp", "GL", "glfw", "sim" }
