## Style Guide

##### General
- stick to the c++11 standard
- explicit over implicit (func(void), return EXIT_SUCCESS, etc.)
- do not pass nonconst references to functions
- avoid pointers but use them to indicate mutating functions
- return type in the line above the function in cpp files

##### Naming conventions
- types CamelCase
- variables snake_case
- file extensions .h .cpp
- filenames snake_case.cpp
- use dashes in cli arguments ```--solver-type=ImplicitEuler```

##### Indentation
- four spaces no tabs
- indent visibility statements
```
    class FileLogger {
        private:
            std::ofstream;
        public:
            FileLogger(std::string const & path);
    }
```
- braces in the
    - same line for keywords (if, for, while, switch, class, etc.)
    - next line for function declarations
- always use braces for control structures
    - exception single keyword statements `if (!condition) return;`

##### Spaces
- spaces around binary operators
- single space precede curly braces
    - exception curly brace is first nonspace character in line

##### Comments and Documentation
TODO
