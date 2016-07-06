local mve = require "mve"
project "evaluate_reconstruction"
    kind "ConsoleApp"
    language "C++"

    buildoptions { "-fopenmp" }
    links { "gomp" }

    files { "evaluate_reconstruction.cpp" }

    mve.use({ "util" })
