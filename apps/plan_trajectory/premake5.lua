local mve = require "mve"

project "plan_trajectory"
    kind "ConsoleApp"
    language "C++"
    toolset "nvcc"

    buildoptions { "-Xcompiler -fopenmp" }

    files { "plan_trajectory.cu" }

    mve.use({ "util" })

    links { "gomp", "fmt", "utp", "eval" }
