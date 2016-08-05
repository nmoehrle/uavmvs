local mve = require "mve"
project "determine_req_matching"
    kind "ConsoleApp"
    language "C++"

    buildoptions { "-fopenmp" }
    links { "gomp" }

    files { "determine_req_matching.cpp" }

    mve.use({ "util" })
