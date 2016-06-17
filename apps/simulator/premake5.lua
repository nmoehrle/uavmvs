local mve = require "mve"

project "simulator"
    kind "ConsoleApp"
    language "C++"

    files { "simulator.cpp" }

    mve.use({ "util", "ogl" })

    links { "GL", "glfw", "sim" }
