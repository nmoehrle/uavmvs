local mve = require "mve"

project "generate_proxy_cloud"
    kind "ConsoleApp"
    language "C++"

    buildoptions { "-fopenmp" }
    links { "gomp" }

    files { "generate_proxy_cloud.cpp" }

    mve.use({ "util" })
