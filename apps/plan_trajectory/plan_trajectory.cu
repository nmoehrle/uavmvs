#include <iostream>

#include "util/system.h"
#include "util/arguments.h"
#include "util/choices.h"

struct Arguments {
    std::string proxy_mesh;
    std::string proxy_cloud;
    std::string aabb;
    float min_distance;
    float max_velocity;
};

Arguments parse_args(int argc, char **argv) {
    util::Arguments args;
    args.set_exit_on_error(true);
    args.set_nonopt_minnum(3);
    args.set_nonopt_maxnum(3);
    args.set_usage("Usage: " + std::string(argv[0])
        + " [OPTS] PROXY_CLOUD AABB");
    args.set_description("Plans a trajectory maximizing reconstructability");
    args.parse(argc, argv);

    Arguments conf;
    conf.proxy_cloud = args.get_nth_nonopt(0);
    conf.aabb = args.get_nth_nonopt(1);

    for (util::ArgResult const* i = args.next_option();
         i != 0; i = args.next_option()) {
        switch (i->opt->sopt) {
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

    return EXIT_SUCCESS;
}
