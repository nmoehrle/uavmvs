/*
 * Copyright (C) 2016-2018, Nils Moehrle
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the LICENSE.txt file for details.
 */

#include <fstream>
#include <cstring>

#include "util/endian.h"

#define NUMPY_FILE_SIGNATURE "\x93NUMPY"
#define NUMPY_FILE_SIGNATURE_LEN 6
#define NUMPY_FILE_VERSION_MAJOR "\x02"
#define NUMPY_FILE_VERSION_MINOR "\x00"

#ifdef HOST_BYTEORDER_LE
#   define HOST_BYTEORDER_SIGN '<'
#else
#   define HOST_BYTEORDER_SIGN '>'
#endif

std::uint32_t divup(std::uint32_t x, std::uint32_t y) {
    return x / y  + (x % y != 0);
}

template <typename T>
std::tuple<std::size_t, std::size_t, const char *>
dimension(std::string const & filename) {
    return std::make_tuple(0, 0, filename.c_str());
}

template <typename T, typename... Args>
std::tuple<std::size_t, std::size_t, const char *>
dimension(std::vector<T> const & vec, Args const &... args)
{
    std::size_t n, m;
    const char * filename;
    std::tie(n, m, filename) = dimension<T>(args...);

    if (n != 0 && vec.size() != m) {
        throw std::runtime_error("Dimension missmatch");
    }

    return std::make_tuple(n + 1, vec.size(), filename);
}

template <typename T>
void write_binary(std::ofstream *, std::string const &) {}

template <typename T, typename... Args>
void write_binary(std::ofstream * out, std::vector<T> const & vec, Args const &... args) {
    out->write(reinterpret_cast<const char*>(vec.data()), vec.size() * sizeof(T));
    write_binary<T>(out, args...);
}

template <typename T, typename... Args>
void save_numpy_file(std::vector<T> const & vec,
    Args const &... args)
{
    static_assert(std::is_arithmetic<T>::value,
        "Arithmetic type required");

    std::size_t n, m;
    const char * filename;
    std::tie(n, m, filename) = dimension(vec, args...);

    char type = std::is_floating_point<T>::value ? 'f' : 'i';

    std::stringstream oss;
    oss << "{"
        << "'descr': '" << HOST_BYTEORDER_SIGN << type << sizeof(T) << "', "
        << "'fortran_order': False, "
        << "'shape': (" << n << ", " << m << "), "
        << "}";
    std::string dict = oss.str();

    std::uint32_t header_length = dict.size() + NUMPY_FILE_SIGNATURE_LEN + 6 + 1;
    std::uint32_t padding = 16u * divup(header_length, 16u) - header_length;
    header_length += padding - NUMPY_FILE_SIGNATURE_LEN - 6;

#ifdef HOST_BYTEODER_BE
    util::byteswap<4>(&header_length);
#endif

    std::ofstream out(filename, std::ios::binary);
    if (!out.good()) throw util::FileException(filename, std::strerror(errno));

    out.write(NUMPY_FILE_SIGNATURE, NUMPY_FILE_SIGNATURE_LEN);
    out.write(NUMPY_FILE_VERSION_MAJOR, 1);
    out.write(NUMPY_FILE_VERSION_MINOR, 1);

    out.write(reinterpret_cast<const char*>(&header_length), 4);
    out.write(dict.c_str(), dict.size());
    out.write("               ", padding);
    out.write("\n", 1);

    write_binary(&out, vec, args...);
    out.close();
}
