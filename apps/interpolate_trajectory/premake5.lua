local mve = require "mve"
project "interpolate_trajectory"
    kind "ConsoleApp"
    language "C++"

    buildoptions { "-fopenmp" }
    sysincludedirs { "/usr/include/eigen3" }
    files { "interpolate_trajectory.cpp" }

    mve.use({ "util" })
    links { "gomp" }
