local mve = require "mve"

project "estimate_capture_difficulty"
    kind "ConsoleApp"
    language "C++"
    toolset "nvcc"

    files { "estimate_capture_difficulty.cu" }

    mve.use({ "util" })

    links { "eval" }
