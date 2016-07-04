#ifndef UTIL_PROGRESSCOUNTER_HEADER
#define UTIL_PROGRESSCOUNTER_HEADER

#include <mutex>
#include <atomic>
#include <fstream>
#include <iostream>
#include <sstream>
#include <cmath>

#include <util/timer.h>

enum ProgressCounterStyle {
    BASIC,
    ETA
};

static const std::string clear = "\r" + std::string(80,' ') + "\r";

class ProgressCounter {
    private:
        std::ofstream tty;
        util::WallTimer timer;

        std::string task;
        std::size_t max;

        std::atomic_size_t count;
        std::mutex mutex;

    public:
        ProgressCounter(std::string const & task, std::size_t max);
        template <ProgressCounterStyle T> void progress(void);
        void inc(void);
        void reset(std::string const & newtask = std::string());
};

inline
ProgressCounter::ProgressCounter(std::string const & task, std::size_t max)
    : tty("/dev/tty", std::ios_base::out), timer(),
    task(task), max(max), count(0) {}

inline void
ProgressCounter::inc(void) {
    std::size_t tmp = ++count;

    if(tmp == max) {
        std::stringstream ss;
        ss << clear << task << " 100%... done. (Took "
            << timer.get_elapsed_sec() << "s)";
        std::lock_guard<std::mutex> guard(mutex);
        std::cout << ss.rdbuf() << std::endl;
    }
}

inline void
ProgressCounter::reset(std::string const & newtask) {
    timer.reset();
    count = 0;
    if (!newtask.empty()) {
        task = newtask;
    }
}

template <ProgressCounterStyle T> void
ProgressCounter::progress(void) {
    if (max <= 100 || count % (max / 100) == 0) {
        float percent = static_cast<float>(count) / max;
        int ipercent = std::floor(percent * 100.0f + 0.5f);

        std::stringstream ss;
        ss << clear << task << " " << ipercent << "%...";

        if (T == ETA && ipercent > 3){
            std::size_t const elapsed = timer.get_elapsed();
            std::size_t eta = (elapsed / percent - elapsed) / 1000;
            ss << " eta ~ " << eta << " s";
        }

        std::lock_guard<std::mutex> guard(mutex);
        tty << ss.rdbuf() << std::flush;
    }
}

#endif /* UTIL_PROGRESSCOUNTER_HEADER */
