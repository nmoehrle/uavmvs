local mve = require "mve"
project "convert-volume"
    kind "ConsoleApp"
    language "C++"

    files { "convert.cpp" }

    mve.use({ "util" })

    links { "fmt" }
