#include <iostream>

#include "util/system.h"
#include "util/arguments.h"
#include "util/choices.h"

enum Choice {
    FIRST_CHOICE = 0,
    SECOND_CHOICE = 1
};

template <> inline
const std::vector<std::string> choice_strings<Choice>() {
    return {"first_choice", "second_choice"};
}

struct Arguments {
    std::string argument;
    std::string first_option;
    float second_option;
    Choice choice;
    bool flag;
};

Arguments parse_args(int argc, char **argv) {
    util::Arguments args;
    args.set_exit_on_error(true);
    args.set_nonopt_minnum(1);
    args.set_nonopt_maxnum(1);
    args.set_usage("Usage: " + std::string(argv[0]) + " [OPTS] ARGUMENT");
    args.set_description("Template app");
    args.add_option('f', "first-option", true, "first option");
    args.add_option('s', "second-option", true, "second option");
    args.add_option('\0', "flag", false, "boolean flag [false]");
    args.add_option('\0', "choice", true, "choices "
            + choices<Choice>(FIRST_CHOICE));
    args.parse(argc, argv);

    Arguments conf;
    conf.argument = args.get_nth_nonopt(0);
    conf.first_option = "";
    conf.second_option = 0.0f;
    conf.choice = FIRST_CHOICE;
    conf.flag = false;

    for (util::ArgResult const* i = args.next_option();
         i != 0; i = args.next_option()) {
        switch (i->opt->sopt) {
        case 'f':
            conf.first_option = i->arg;
        break;
        case 's':
            conf.second_option = i->get_arg<float>();
        break;
        case '\0':
            if (i->opt->lopt == "choice") {
                conf.choice = parse_choice<Choice>(i->arg);
            } else if (i->opt->lopt == "flat") {
                conf.flag = true;
            } else {
                throw std::invalid_argument("Invalid option");
            }
        break;
        default:
            throw std::invalid_argument("Invalid option");
        }
    }

    return conf;
}

int main(int argc, char **argv) {
    util::system::register_segfault_handler();
    util::system::print_build_timestamp(argv[0]);

    Arguments args = parse_args(argc, argv);

    /* Here you go */

    return EXIT_SUCCESS;
}
