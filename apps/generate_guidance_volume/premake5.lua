local mve = require "mve"

project "generate_guidance_volume"
    kind "ConsoleApp"
    language "C++"

    toolset "nvcc"
    buildoptions { "-Xcompiler -fopenmp" }

    files { "generate_guidance_volume.cu" }

    mve.use({ "util" })

    links { "gomp", "fmt", "eval" }
