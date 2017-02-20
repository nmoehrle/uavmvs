local mve = require "mve"

project "generate_gcp_bundle"
    kind "ConsoleApp"
    language "C++"

    buildoptions { "-fopenmp" }

    files { "generate_gcp_bundle.cpp" }

    mve.use({ "util", "sfm" })

    links { "gomp" }
