local mve = require "mve"

project "template"
    kind "ConsoleApp"
    language "C++"

    files { "template.cpp" }

    mve.use({ "util" })
