local mve = require "mve"

project "generate-trajectory"
    kind "ConsoleApp"
    language "C++"

    files { "generate.cpp" }

    mve.use({ "util" })
    links { "utp" }

project "shorten-trajectory"
    kind "ConsoleApp"
    language "C++"

    buildoptions { "-fopenmp" }
    files { "shorten.cpp" }

    mve.use({ "util" })

    links { "gomp", "utp" }

project "interpolate-trajectory"
    kind "ConsoleApp"
    language "C++"

    sysincludedirs { "/usr/include/eigen3" }
    files { "interpolate.cpp" }

    mve.use({ "util" })
    links { "utp" }

project "evaluate-trajectory"
    kind "ConsoleApp"
    language "C++"

    sysincludedirs { "/usr/include/eigen3" }
    files { "evaluate.cpp" }

    mve.use({ "util" })
    links { "utp" }
