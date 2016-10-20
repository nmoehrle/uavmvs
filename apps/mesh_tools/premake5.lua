local mve = require "mve"

project "convert-mesh"
    kind "ConsoleApp"
    language "C++"

    files { "convert.cpp" }

    mve.use({ "util" })

project "normalize-mesh"
    kind "ConsoleApp"
    language "C++"

    files { "normalize.cpp" }

    mve.use({ "util" })

project "colorize-mesh"
    kind "ConsoleApp"
    language "C++"

    files { "colorize.cpp" }

    mve.use({ "util" })

project "estimate_transform-mesh"
    kind "ConsoleApp"
    language "C++"

    buildoptions { "-fopenmp" }
    files { "estimate_transform.cpp" }

    mve.use({ "util" })

    links { "gomp" }
