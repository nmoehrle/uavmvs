local mve = require "mve"

project "generate_point_cloud"
    kind "ConsoleApp"
    language "C++"

    buildoptions { "-fopenmp" }
    links { "gomp" }

    files { "generate_point_cloud.cpp" }

    mve.use({ "util" })
