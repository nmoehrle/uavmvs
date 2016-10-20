local mve = require "mve"

project "estimate_transform-scene"
    kind "ConsoleApp"
    language "C++"

    buildoptions { "-fopenmp" }

    files { "estimate_transform.cpp" }

    mve.use({ "util" })

    links { "gomp" }
