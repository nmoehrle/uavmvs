local mve = require "mve"

project "convert-scene"
    kind "ConsoleApp"
    language "C++"

    files { "convert.cpp" }

    mve.use({ "util" })

project "evaluate-scene"
    kind "ConsoleApp"
    language "C++"

    files { "evaluate.cpp" }

    mve.use({ "util" })

project "annotate-scene"
    kind "ConsoleApp"
    language "C++"

    files { "annotate.cpp" }

    mve.use({ "util" })

project "estimate_transform-scene"
    kind "ConsoleApp"
    language "C++"

    buildoptions { "-fopenmp" }

    files { "estimate_transform.cpp" }

    mve.use({ "util" })

    links { "gomp" }

project "reduce_randomly"
    kind "ConsoleApp"
    language "C++"

    files { "reduce_randomly.cpp" }

    mve.use({ "util" })

project "deduce_bundle"
    kind "ConsoleApp"
    language "C++"

    buildoptions { "-fopenmp" }

    files { "deduce_bundle.cpp" }

    mve.use({ "util", "sfm" })

    links { "gomp" }

project "match-scene"
    kind "ConsoleApp"
    language "C++"

    files { "match.cpp" }

    mve.use({ "util" })
