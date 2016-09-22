local mve = require "mve"
project "convert-volume"
    kind "ConsoleApp"
    language "C++"

    buildoptions { "-fopenmp" }
    files { "convert.cpp" }

    mve.use({ "util" })

    links { "fmt", "gomp" }

project "normalize-volume"
    kind "ConsoleApp"
    language "C++"

    files { "normalize.cpp" }

    mve.use({ "util" })
