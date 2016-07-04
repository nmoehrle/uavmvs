local mve = require "mve"

project "capture_trajectory"
    kind "ConsoleApp"
    language "C++"

    files { "capture_trajectory.cpp" }

    mve.use({ "util", "ogl" })

    links { "GL", "glfw", "sim" }
