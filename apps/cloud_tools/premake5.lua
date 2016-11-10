local mve = require "mve"

project "convert-cloud"
    kind "ConsoleApp"
    language "C++"

    files { "convert.cpp" }

    mve.use({ "util" })
