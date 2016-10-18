local mve = require "mve"

project "determine_transform"
    kind "ConsoleApp"
    language "C++"

    buildoptions { "-fopenmp" }

    files { "determine_transform.cpp" }

    mve.use({ "util" })

    links { "gomp" }
