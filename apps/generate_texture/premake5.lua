local mve = require "mve"
local mvst = require "mvst"

project "generate_texture"
    kind "ConsoleApp"
    language "C++"

    buildoptions { "-fopenmp" }

    files { "generate_texture.cpp", "simplex_noise.cpp" }

    mve.use({ "util" })
    mvst.use({ "tex" })

    links { "gomp" }
