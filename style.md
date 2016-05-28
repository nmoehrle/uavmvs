## Style Guide

##### General
Stick to the c++11 standard
Explicit over implicit (func(void), return EXIT_SUCCESS, etc.)
Do not pass nonconst references to functions
Avoid pointers but use them to indicate mutating functions
Return type in the line above the function in cpp files

##### Naming conventions
Types CamelCase
Variables snake_case
Filenames snake_case.h snake_case.cpp
Use dashes in cli arguments --solver-type=ImplicitEuler

##### Indentation
Four spaces no tabs
Indent visibility statements
'''
    class FileLogger {
        private:
            std::ofstream;
        public:
            FileLogger(std::string const & path);
    }
'''
Braces in the
    - same line for keywords (if, for, while, switch, class, etc.)
    - next line for function declarations
Allways use braces for control structures except if the only statement is a keyword
    'if (!condition) return;'

##### Comments and Documentation
TODO
