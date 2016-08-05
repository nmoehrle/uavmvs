local mve = require "mve"
project "generate_proxy_mesh"
    kind "ConsoleApp"
    language "C++"

    buildoptions { "-fopenmp" }

    files { "generate_proxy_mesh.cpp" }
    links { "fmt", "gomp" }

    mve.use({ "util", "fssr" })
