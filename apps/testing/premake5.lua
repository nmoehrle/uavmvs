local mve = require "mve"

project "interop"
    kind "ConsoleApp"
    toolset "nvcc"
    language "C++"

    files { "interop.cu" }

    mve.use({ "util", "ogl" })

    links { "GL", "glfw", "sim" }
