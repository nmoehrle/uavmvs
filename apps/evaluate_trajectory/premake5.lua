local mve = require "mve"

project "evaluate_trajectory"
    kind "ConsoleApp"
    language "C++"
    toolset "nvcc"

    buildoptions { "-Xcompiler -fopenmp" }

    files { "evaluate_trajectory.cu" }

    mve.use({ "util" })

    links { "gomp", "utp", "eval" }
