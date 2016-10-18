local mve = require "mve"

project "determine_matching_mask"
    kind "ConsoleApp"
    language "C++"

    buildoptions { "-fopenmp" }

    files { "determine_matching_mask.cpp" }

    mve.use({ "util" })

    links { "gomp" }
