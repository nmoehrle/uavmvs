local mve = require "mve"

project "evaluate_heuristic"
    kind "ConsoleApp"
    language "C++"
    toolset "nvcc"

    buildoptions { "-Xcompiler -fopenmp" }

    files { "evaluate_heuristic.cu" }

    mve.use({ "util" })

    links { "gomp", "eval" }
