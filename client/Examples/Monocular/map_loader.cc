/**
* This file is not part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<opencv2/core/core.hpp>
#include <BoostArchiver.h>

#include <Map.h>
#include <Viewer.h>
#include <KeyFrameDatabase.h>

using namespace std;
using namespace ORB_SLAM2;

void loadMap(const string &filename, Map *&pMap, KeyFrameDatabase *pKeyFrameDatabase) {
    std::ifstream in(filename, std::ios_base::binary);
    if (!in) {
        cerr << "Cannot Open Mapfile: " << filename << " , Create a new one" << std::endl;
        return;
    }
    cout << "Loading Mapfile: " << filename << std::flush;
    boost::archive::binary_iarchive ia(in, boost::archive::no_header);
    ia >> pMap;
    ia >> pKeyFrameDatabase;
//    mpKeyFrameDatabase->SetORBvocabulary(mpVocabulary);
    cout << " ...done" << std::endl;
    cout << "Map Reconstructing" << flush;
//    vector<ORB_SLAM2::KeyFrame *> vpKFS = mpMap->GetAllKeyFrames();
//    for (auto it:vpKFS) {
//        it->SetORBvocabulary(mpVocabulary);
//        it->ComputeBoW();
//    }
    cout << " ...done" << endl;
    in.close();
}

int main(int argc, char **argv) {
    if (argc != 3) {
        cerr << endl << "Usage: ./map_load map_file_path setting_file_path" << endl;
        return 1;
    }


    const string filename = argv[1];
    Map *pMap = new Map();
    KeyFrameDatabase *pkfDB = new KeyFrameDatabase();

#ifdef COMPILEDWITHC11
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
    std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

    loadMap(filename, pMap, pkfDB);
#ifdef COMPILEDWITHC11
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
    std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

    double ttrack = std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

//    vTimesTrack[ni] = ttrack;

    cout << "map load time: " << ttrack << endl;

    for (const auto &kf: pMap->mspKeyFrames) {
        if (!kf) continue;
        cout << "keyframe: " << kf->mnId << endl;
        for (const auto &mp: kf->mvpMapPoints) {
            if (!mp) continue;
            cout << "mappoint: " << mp->mnId << endl;
        }
    }

    auto pFrameDrawer = new FrameDrawer(pMap);
    auto pMapDrawer = new MapDrawer(pMap, argv[2]);
    auto pViewer = new Viewer(nullptr, pFrameDrawer, pMapDrawer, nullptr, argv[2]);
    pViewer->Run();

    cv::waitKey();

    return 0;
}

