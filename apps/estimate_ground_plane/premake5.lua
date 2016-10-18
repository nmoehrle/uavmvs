local mve = require "mve"

project "estimate_ground_plane"
    kind "ConsoleApp"
    language "C++"

    buildoptions { "-fopenmp" }

    files { "estimate_ground_plane.cpp" }

    mve.use({ "util" })

    links { "gomp" }
