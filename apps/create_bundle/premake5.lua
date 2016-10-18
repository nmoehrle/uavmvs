local mve = require "mve"

project "create_bundle"
    kind "ConsoleApp"
    language "C++"

    buildoptions { "-fopenmp" }

    files { "create_bundle.cpp" }

    mve.use({ "util" })

    links { "gomp" }
