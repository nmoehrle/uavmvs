local mve = require "mve"

project "generate-trajectory"
    kind "ConsoleApp"
    language "C++"

    files { "generate.cpp" }

    links { "utp" }
    mve.use({ "util" })
