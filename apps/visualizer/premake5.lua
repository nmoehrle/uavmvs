local mve = require "mve"

project "visualizer"
    kind "ConsoleApp"
    language "C++"

    sysincludedirs { "/usr/include/eigen3" }
    files { "visualizer.cpp" }

    mve.use({ "util", "ogl" })

    links { "GL", "glfw", "sim", "utp" }
