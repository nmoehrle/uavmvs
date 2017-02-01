local mve = require "mve"

project "evaluate_ground_sampling"
    kind "ConsoleApp"
    language "C++"

    buildoptions { "-fopenmp" }

    files { "evaluate_ground_sampling.cpp" }

    mve.use({ "util" })

    links { "gomp" }
