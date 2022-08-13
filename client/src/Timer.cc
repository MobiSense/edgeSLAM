//
// Created by Halcao on 2020/5/26.
//

#include <CLogger.h>
#include "Timer.h"

namespace ORB_SLAM2 {
    Timer::Timer(bool _logTime): logTime(_logTime), start(std::chrono::steady_clock::now()) {}

    double Timer::get() {
        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
        return std::chrono::duration_cast<std::chrono::duration<double> >(end - start).count();
    }

    Timer::~Timer() {
        if (logTime) {
            trace("track time: {}", get());
        }
    }
};