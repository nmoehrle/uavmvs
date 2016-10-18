local mve = require "mve"
project "generate_grid_trajectory"
    kind "ConsoleApp"
    language "C++"

    files { "generate_grid_trajectory.cpp" }

    mve.use({ "util" })

    links { "utp" }
