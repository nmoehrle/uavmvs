#ifndef GEOM_VOLUME_IO_HEADER
#define GEOM_VOLUME_IO_HEADER

#include <cstring>
#include <fstream>

#include "util/exception.h"

#include "volume.h"

#define GEOM_VOLUME_FILE_HEADER "VOL"
#define GEOM_VOLUME_FILE_VERSION "0.1"

template <typename IdxType>
void save_volume(typename Volume<IdxType>::ConstPtr volume, std::string const & filename) {
    std::ofstream out(filename.c_str(), std::ios::binary);
    if (!out.good()) {
        throw util::FileException(filename, std::strerror(errno));
    }

    out << GEOM_VOLUME_FILE_HEADER << " "
        << GEOM_VOLUME_FILE_VERSION << std::endl;
    out << volume->width() << " "
        << volume->height() << " "
        << volume->depth() << " " << std::endl;
    out << volume->minimum() << " " << volume->maximum();

    for (IdxType i = 0; i < volume->num_positions(); ++i) {
        mve::FloatImage::ConstPtr image = volume->at(i);
        if (image == nullptr) continue;
        out << std::endl << i << " "
            << image->width() << " "
            << image->height() << " "
            << image->channels() << std::endl;
        out.write(image->get_byte_pointer(), image->get_byte_size());
    }
    out.close();
}

template <typename IdxType>
typename Volume<IdxType>::Ptr load_volume(const std::string & filename) {
    std::ifstream in(filename.c_str(), std::ios::binary);
    if (!in.good()) {
        throw util::FileException(filename, std::strerror(errno));
    }

    std::string header;

    in >> header;

    if (header != GEOM_VOLUME_FILE_HEADER) {
        in.close();
        throw util::FileException(filename, "Not a Volume file");
    }

    std::string version;
    in >> version;

    if (version != GEOM_VOLUME_FILE_VERSION) {
        in.close();
        throw util::FileException(filename, "Incompatible version of Volume file");
    }

    std::string buffer;
    /* Discard the rest of the line. */
    std::getline(in, buffer);

    IdxType width, height, depth;
    in >> width >> height >> depth;
    std::getline(in, buffer);

    math::Vec3f min, max;
    for (int i = 0; i < 3; ++i) in >> min[i];
    for (int i = 0; i < 3; ++i) in >> max[i];

    if (in.fail()) {
        in.close();
        throw util::FileException(filename, "Corrupt Volume file header");
    }
    std::getline(in, buffer);

    typename Volume<IdxType>::Ptr volume;
    volume = Volume<IdxType>::create(width, height, depth, min, max);

    IdxType idx = -1, channels = 0;
    int iwidth = 0, iheight = 0;
    while(in >> idx >> iwidth >> iheight >> channels) {
        std::getline(in, buffer);
        mve::FloatImage::Ptr image = mve::FloatImage::create(iwidth, iheight, channels);
        if (!(in.read(image->get_byte_pointer(), image->get_byte_size()))) {
            in.close();
            throw util::FileException(filename, "Corrupt Volume file");
        }
        volume->at(idx) = image;
    }
    in.close();

    return volume;
}

#endif /* GEOM_VOLUME_IO_HEADER */
