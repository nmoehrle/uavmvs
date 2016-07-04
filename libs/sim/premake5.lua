local mve = require "mve"

project "sim"
    kind "StaticLib"
    language "C++"

    files {
        "window.cpp",
        "model.cpp"
    }

    mve.use({ "ogl" })

    links { "GL", "glfw" }
