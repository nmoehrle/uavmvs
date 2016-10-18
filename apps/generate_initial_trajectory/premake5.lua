local mve = require "mve"

project "generate_initial_trajectory"
    kind "ConsoleApp"
    language "C++"

    files { "generate_initial_trajectory.cpp" }

    mve.use({ "util" })

    links { "utp" }
