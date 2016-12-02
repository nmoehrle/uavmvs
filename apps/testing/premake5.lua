local mve = require "mve"

project "interop"
    kind "ConsoleApp"
    toolset "nvcc"
    language "C++"

    flags { "RelocatableDeviceCode" }
    buildoptions { "-Xcompiler -fopenmp" }

    files {
        "interop.cu",
        "../../libs/cacc/bvh_tree.cu",
        "../../libs/cacc/tracing.cu",
    }

    mve.use({ "util", "ogl" })

    links { "gomp", "GL", "glfw", "sim" }
