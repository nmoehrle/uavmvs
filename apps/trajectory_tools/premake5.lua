local mve = require "mve"

project "generate-trajectory"
    kind "ConsoleApp"
    language "C++"

    files { "generate.cpp" }

    links { "utp" }
    mve.use({ "util" })

project "shorten-trajectory"
    kind "ConsoleApp"
    language "C++"

    buildoptions { "-fopenmp" }
    sysincludedirs { "/usr/include/eigen3" }
    files { "shorten.cpp" }

    mve.use({ "util" })

    links { "gomp", "utp" }

project "interpolate-trajectory"
    kind "ConsoleApp"
    language "C++"

    sysincludedirs { "/usr/include/eigen3" }
    files { "interpolate.cpp" }

    links { "utp" }
    mve.use({ "util" })
