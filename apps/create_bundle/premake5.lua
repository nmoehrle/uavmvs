local mve = require "mve"
project "create_bundle"
    kind "ConsoleApp"
    language "C++"

    buildoptions { "-fopenmp" }
    links { "gomp" }

    files { "create_bundle.cpp" }

    mve.use({ "util" })
