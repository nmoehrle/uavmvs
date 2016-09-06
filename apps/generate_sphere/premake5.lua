local mve = require "mve"
project "generate_sphere"
    kind "ConsoleApp"
    language "C++"

    files { "generate_sphere.cpp" }

    mve.use({ "util" })
