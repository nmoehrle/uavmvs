local mve = require "mve"

project "convert-scene"
    kind "ConsoleApp"
    language "C++"

    files { "convert.cpp" }

    mve.use({ "util" })

project "estimate_transform-scene"
    kind "ConsoleApp"
    language "C++"

    buildoptions { "-fopenmp" }

    files { "estimate_transform.cpp" }

    mve.use({ "util" })

    links { "gomp" }

project "reduce_randomly"
    kind "ConsoleApp"
    language "C++"

    files { "reduce_randomly.cpp" }

    mve.use({ "util" })
