#include <iostream>
#include <vector>
#include <unordered_map>
#include <algorithm>

#include "geom/volume_io.h"

#include "util/arguments.h"
#include "util/file_system.h"
#include "util/tokenizer.h"

struct Arguments {
    bool clamp;
    std::string in_volume;
    std::string out_volume;
    float eps;
    float no_value;
    std::vector<std::string> volumes;
};

Arguments parse_args(int argc, char **argv) {
    util::Arguments args;
    args.set_exit_on_error(true);
    args.set_nonopt_maxnum(2);
    args.set_nonopt_minnum(2);
    args.set_usage("Usage: " + std::string(argv[0]) + " [OPTS] IN_VOLUME OUT_VOLUME");
    args.set_description("Normalizes the volume values.");
    args.add_option('c', "clamp", false, "clamp (instead of remove) outliers");
    args.add_option('e', "epsilon", true, "remove outliers in percent [0.0]");
    args.add_option('i', "ignore", true, "set value to ignore [-1.0]");
    args.add_option('\0', "meshes", true, "calculate normalization based on these volumes (comma seperated list)."
        "If no mesh is given the normalization is calculate from IN_VOLUME");
    args.parse(argc, argv);

    Arguments conf;
    conf.in_volume = args.get_nth_nonopt(0);
    conf.out_volume = args.get_nth_nonopt(1);
    conf.eps = 0.0f;
    conf.clamp = false;
    conf.no_value = -1.0f;

    for (util::ArgResult const* i = args.next_option();
         i != 0; i = args.next_option()) {
        switch (i->opt->sopt) {
        case 'e':
            conf.eps = i->get_arg<float>();
        break;
        case 'c':
            conf.clamp = true;
        break;
        case 'i':
            conf.no_value = i->get_arg<float>();
        break;
        case '\0':
            if (i->opt->lopt == "volumes") {
                util::Tokenizer t;
                t.split(i->arg, ',');
                conf.volumes = t;
            } else {
                throw std::invalid_argument("Invalid option");
            }
        break;
        default:
            throw std::invalid_argument("Invalid option");
        }
    }

    if (conf.eps < 0.0f || conf.eps > 1.0f) {
        throw std::invalid_argument("epsilon is supposed to be in the intervall [0.0, 1.0]");
    }

    if (conf.volumes.empty()) {
        conf.volumes.push_back(conf.in_volume);
    }

    return conf;
}

int main(int argc, char **argv) {
    Arguments args = parse_args(argc, argv);

    Volume<std::uint32_t>::Ptr volume_to_normalize;

    std::unordered_map<std::string, Volume<std::uint32_t>::Ptr> volumes_to_load;
    for (std::size_t i = 0; i < args.volumes.size(); i++){
        Volume<std::uint32_t>::Ptr volume;
        std::string name = args.volumes[i];
        volumes_to_load[name] = volume;
    }
    volumes_to_load[args.in_volume] = volume_to_normalize;

    std::size_t num_values = 0;
    std::unordered_map<std::string, Volume<std::uint32_t>::Ptr>::iterator it;
    for (it = volumes_to_load.begin(); it != volumes_to_load.end(); it++){
        Volume<std::uint32_t>::Ptr volume;
        try {
            volume = load_volume<std::uint32_t>(it->first);
        } catch (std::exception& e) {
            std::cerr << "Could not load volume: "<< e.what() << std::endl;
            std::exit(EXIT_FAILURE);
        }

        for (std::uint32_t i = 0; i < volume->num_positions(); ++i) {
            mve::FloatImage::Ptr image = volume->at(i);
            if (image != nullptr) {
                num_values += image->get_value_amount();
            }
        }

        it->second = volume;
    }
    volume_to_normalize = volumes_to_load[args.in_volume];

    std::vector<float> values;
    values.reserve(num_values);

    for (std::size_t i = 0; i < args.volumes.size(); ++i) {
        Volume<std::uint32_t>::Ptr volume = volumes_to_load[args.volumes[i]];
        for (std::uint32_t j = 0; j < volume->num_positions(); ++j) {
            mve::FloatImage::Ptr image = volume->at(j);
            if (image == nullptr) continue;

            for (int k = 0; k < image->get_value_amount(); ++k) {
                float value = image->at(k);

                if (value == args.no_value) continue;

                values.push_back(value);
            }
        }
    }

    std::cout << values.size() << " valid values" << std::endl;
    std::size_t c = (values.size() * args.eps) / 2;

    typedef std::vector<float>::iterator Iter;

    std::pair<Iter, Iter> real_minmax = std::minmax_element(values.begin(), values.end());

    float real_min = *(real_minmax.first);
    float real_max = *(real_minmax.second);

    Iter nth;

    nth = values.begin() + c;
    std::nth_element(values.begin(), nth, values.end(), std::less<float>());
    float min = *nth;

    nth = values.begin() + c;
    std::nth_element(values.begin(), nth, values.end(), std::greater<float>());
    float max = *nth;

    float delta = max - min;
    std::cout << "Minimal value: " << real_min << std::endl;
    std::cout << "Maximal value: " << real_max << std::endl;
    std::cout << "Normalizing range " << min << " - " << max << std::endl;

    int num_outliers = 0;
    for (std::uint32_t i = 0; i < volume_to_normalize->num_positions(); ++i) {
        mve::FloatImage::Ptr image = volume_to_normalize->at(i);
        if (image == nullptr) continue;

        for (int j = 0; j < image->get_value_amount(); ++j) {
            float value = image->at(j);
            if (value == args.no_value) continue;

            if (value >= min) {
                if(value <= max) {
                    image->at(j) = ((value - min) / delta);
                } else {
                    image->at(j) = args.clamp ? 1.0f : args.no_value;
                    num_outliers++;
                }
            } else {
                image->at(j) = args.clamp ? 0.0f : args.no_value;
                num_outliers++;
            }
        }
    }

    if (args.clamp) {
        std::cout << "Clamped ";
    } else {
        std::cout << "Removed ";
    }
    std::cout << num_outliers << " outliers" << std::endl;

    save_volume<std::uint32_t>(volume_to_normalize, args.out_volume);
}
