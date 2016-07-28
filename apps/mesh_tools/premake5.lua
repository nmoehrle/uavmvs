local mve = require "mve"
project "prepare_mesh"
    kind "ConsoleApp"
    language "C++"

    files { "prepare_mesh.cpp" }

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
