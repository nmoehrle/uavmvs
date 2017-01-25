local mve = require "mve"

project "convert-image"
    kind "ConsoleApp"
    language "C++"

    files { "convert.cpp" }

    mve.use({ "util" })

project "normalize-image"
    kind "ConsoleApp"
    language "C++"

    files { "normalize.cpp" }

    mve.use({ "util" })

project "colorize-image"
    kind "ConsoleApp"
    language "C++"

    files { "colorize.cpp" }

    mve.use({ "util" })
