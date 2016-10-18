local mve = require "mve"
project "visualizer"
    kind "ConsoleApp"
    language "C++"

    files { "visualizer.cpp" }

    mve.use({ "util", "ogl" })

    links { "GL", "glfw", "sim", "utp" }
