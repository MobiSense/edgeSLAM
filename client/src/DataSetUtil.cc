//
// Created by Halcao on 2020/5/17.
//

#include "DataSetUtil.h"
#include <sstream>
#include <fstream>

namespace ORB_SLAM2 {
    void DataSetUtil::LoadEuRoC(const std::string &strImagePath, const std::string &strPathTimes,
                                std::vector<std::string> &vstrImages, std::vector<double> &vTimeStamps) {
        std::ifstream fTimes;
        fTimes.open(strPathTimes.c_str());
        vTimeStamps.reserve(5000);
        vstrImages.reserve(5000);
        while (!fTimes.eof()) {
            std::string s;
            getline(fTimes, s);
            if (!s.empty()) {
                std::stringstream ss;
                ss << s;
                vstrImages.push_back(strImagePath + "/" + ss.str() + ".png");
                double t;
                ss >> t;
                vTimeStamps.push_back(t / 1e9);
            }
        }
    }

    void DataSetUtil::LoadTUM(const std::string &path, std::vector<std::string> &vstrImageFilenames,
                    std::vector<double> &vTimestamps) {
        std::string strFile = path + "/rgb.txt";
        std::ifstream f;
        f.open(strFile.c_str());

        // skip first three lines
        std::string s0;
        getline(f, s0);
        getline(f, s0);
        getline(f, s0);

        std::string filename;
        while (!f.eof()) {
            std::string s;
            getline(f, s);
            if (!s.empty()) {
                std::stringstream ss;
                ss << s;
                double t;
                std::string sRGB;
                ss >> t;
                vTimestamps.push_back(t);
                ss >> sRGB;
                filename = path + "/" + sRGB;
                vstrImageFilenames.push_back(filename);
            }
        }
    }
}