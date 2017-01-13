local mve = require "mve"

project "tune_heuristic"
    kind "ConsoleApp"
    language "C++"
    toolset "nvcc"

    buildoptions { "-Xcompiler -fopenmp" }

    files { "tune_heuristic.cu" }

    mve.use({ "util" })

    links { "gomp", "eval" }
