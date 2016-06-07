local mve = require "mve"

project "generate_point_cloud"
    kind "ConsoleApp"
    language "C++"

    files { "generate_point_cloud.cpp" }

    mve.use({ "util" })
