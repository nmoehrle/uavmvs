local mve = require "mve"

project "generate_sphere_mesh"
    kind "ConsoleApp"
    language "C++"

    files { "generate_sphere_mesh.cpp" }

    mve.use({ "util" })

project "generate_aabb_mesh"
    kind "ConsoleApp"
    language "C++"

    files { "generate_aabb_mesh.cpp" }

    mve.use({ "util" })
