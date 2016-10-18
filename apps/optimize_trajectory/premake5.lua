local mve = require "mve"

project "optimize_trajectory"
    kind "ConsoleApp"
    language "C++"
    toolset "nvcc"

    buildoptions { "-Xcompiler -fopenmp" }

    files { "optimize_trajectory.cu" }

    mve.use({ "util" })

    links { "gomp", "utp", "eval" }
