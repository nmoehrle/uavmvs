#ifndef UTIL_ITOS_HEADER
#define UTIL_ITOS_HEADER

std::string
litos(std::uint64_t n) {
    /* len('{:,}'.format(2**64)) + 1 == 27 */
    static char b[27];

    char * it = b;
    for (int i = 0;; ++i) {
        *it++ = char(n % 10ull) + 48;
        if (!(n = n / 10ull)) break;
        if (i == 2) {
            *it++ = ',';
            i = -1;
        }
    }

    *it-- = 0;

    char * jt = b;
    while (jt < it) {
        char t = *jt;
        *jt++ = *it;
        *it-- = t;
    }

    return std::string(b);
}

#endif /* UTIL_ITOS_HEADER */
