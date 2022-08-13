//
// Created by halcao on 19-7-9.
//


#ifndef EDGE_SLAM_GRPCCONFIG_H
#define EDGE_SLAM_GRPCCONFIG_H

enum ServicePort {
    LoopClosingPort,
    OptimizerPort,
    GlobalPort,
    LocalMappingPort,
    TrackingPort,
};

class Config {
public:
    string ServerAddr;
    string ClientAddr;

    int LoopClosing;
    int Optimizer;
    int Global;
    int LocalMapping;
    int Tracking;

    static Config &instance() {
        static Config instance;
        return instance;
    }

    void operator()(Config const&) = delete;

    static void load(string filename) {
        cv::FileStorage fSettings(filename, cv::FileStorage::READ);
        fSettings["Client.Addr"] >> instance().ClientAddr;
        fSettings["Server.Addr"] >> instance().ServerAddr;
        fSettings["Server.LoopClosingPort"] >> instance().LoopClosing;
        fSettings["Server.OptimizerPort"] >> instance().Optimizer;
        fSettings["Server.GlobalPort"] >> instance().Global;
        fSettings["Server.LocalMappingPort"] >> instance().LocalMapping;
        fSettings["Client.TrackingPort"] >> instance().Tracking;

    }

    static string GetAddress(bool server, ServicePort port) {
        string addr = (server ? instance().ServerAddr : instance().ClientAddr) + ":";
        switch (port) {
            case LoopClosingPort:
                return addr + to_string(instance().LoopClosing);
            case OptimizerPort:
                return addr + to_string(instance().Optimizer);
            case GlobalPort:
                return addr + to_string(instance().Global);
            case LocalMappingPort:
                return addr + to_string(instance().LocalMapping);
            case TrackingPort:
                return addr + to_string(instance().Tracking);
        }
    }

private:
    Config(Config const&);
    Config() { }
};

#endif //EDGE_SLAM_GRPCCONFIG_H
