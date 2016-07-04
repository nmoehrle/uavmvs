local mve = require "mve"
project "prepare_mesh"
    kind "ConsoleApp"
    language "C++"

    files { "prepare_mesh.cpp" }

    mve.use({ "util" })

project "normalize_mesh_values"
    kind "ConsoleApp"
    language "C++"

    files { "normalize_mesh_values.cpp" }

    mve.use({ "util" })

project "colorize_mesh"
    kind "ConsoleApp"
    language "C++"

    files { "colorize_mesh.cpp" }

    mve.use({ "util" })
