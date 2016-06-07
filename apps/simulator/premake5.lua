local mve = require "mve"

project "simulator"
    kind "ConsoleApp"
    language "C++"

    files { "simulator.cpp", "window.cpp" }

    mve.use({ "util", "ogl" })

    links { "GL", "glfw" }
