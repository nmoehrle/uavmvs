local mve = require "mve"
project "prepare_mesh"
    kind "ConsoleApp"
    language "C++"

    files { "prepare_mesh.cpp" }

    mve.use({ "util" })
