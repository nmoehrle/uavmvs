local mve = require "mve"

project "selector"
    kind "ConsoleApp"
    language "C++"

    buildoptions { "-fopenmp" }
    files { "selector.cpp" }

    mve.use({ "util", "ogl" })

    links { "GL", "glfw", "sim", "gomp" }
