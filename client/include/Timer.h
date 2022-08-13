//
// Created by Halcao on 2020/5/26.
//

#ifndef EDGE_SLAM_TIMER_H
#define EDGE_SLAM_TIMER_H
#include<chrono>

namespace ORB_SLAM2 {
    class Timer {
    public:
        Timer(bool logTime=true);
        ~Timer();

        double get();
    private:
        bool logTime = true;
        std::chrono::steady_clock::time_point start;
    };
}

#endif //EDGE_SLAM_TIMER_H
