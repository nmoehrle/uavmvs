local mve = require "mve"

project "utp"
    kind "StaticLib"
    language "C++"

    sysincludedirs { "/usr/include/eigen3" }
    files {
        "trajectory.cpp",
        "trajectory_io.cpp"
    }

    mve.use()
