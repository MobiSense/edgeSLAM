/**
* This file is part of ORB-SLAM2.
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

#include<System.h>
#include <Map.h>
#include <KeyFrameDatabase.h>
#include <future>
//#include <boost/archive/binary_oarchive.hpp>
//#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <BoostArchiver.h>
#include <MapUpdater.h>
#include <Client.h>
#include <Timer.h>
#include <MapManager.h>
#include <Converter.h>
#include "CLogger.h"
#include "DataSetUtil.h"

using namespace std;
using namespace ORB_SLAM2;


vector<ORB_SLAM2::System *> SLAMs;
vector<ORB_SLAM2::Client *> clients;
ORB_SLAM2::Client *globalClient;

std::atomic_bool b;

void SerializeMap(Map *pMap, string &result) {
//    std::stringstream oss;
//    boost::archive::text_oarchive oa(oss);
//    boost::archive::binary_oarchive oa(oss);

    auto filename = "tmp.bin";
    std::ofstream out(filename);
    boost::archive::text_oarchive oa(out);
//    unique_lock<mutex> lock(SLAM->GetMap()->mMutexMap);
    pMap->ArchiveMap(oa);
//    result = oss.str();

    info("Map {} serialized, size: {} bytes -- {:03.2f} MB", pMap->mnId,
         result.size(), double(result.size()) / 1024.0 / 1024.0);
}

void UpdateMap(Map *pMap, const string &result) {
//    std::stringstream iss(result);
//    boost::archive::text_iarchive ia(iss);
//            boost::archive::binary_iarchive ia(iss);

    auto filename = "tmp.bin";
    std::ifstream ifs(filename, std::ios::binary);
    boost::archive::text_iarchive ia(ifs);

    pMap->UpdateMap(ia);

    info("Map {} deserialized, size: {} bytes -- {:03.2f} MB", pMap->mnId,
         result.size(), double(result.size()) / 1024.0 / 1024.0);
}

void ReceiveMap(ORB_SLAM2::System *SLAM, ORB_SLAM2::Client *client) {
    info("receive map called");
    if (!SLAM || !SLAM->GetMap() || !client || !client->GetMap()) return;
    // simulate sending and receiving process

    Timer t(true);

    string result;
    SerializeMap(SLAM->GetMap(), result);
    UpdateMap(client->GetMap(), result);

    auto group = MapManager::GetGroup(client->GetMap());
    auto clientMapId = client->GetMap()->mnId;
    Map *pMap = nullptr;
    for (auto &m: group) {
        if (clientMapId != m->mnId) {
            pMap = m;
            break;
        }
    }

    if (!pMap) return;


    // pMap is not base, convert its coordinate into base(use it's global pos)
    auto KFs = pMap->GetAllKeyFrames();

    auto t12 = client->GetMap()->mTwl.inverse() * pMap->mTwl;

    // TODO(halcao): send only selected frame poses
    for (auto &kf: KFs) {
        cv::Mat newPose = kf->GetPose() * Converter::toCvMat(t12.inverse()) * t12.scale();
        pMap->AddUpdate(new KeyFrameUpdate<cv::Mat>(kf->mnId, "SetPose", newPose));
    }

    auto MPs = pMap->GetAllMapPoints();
    for (auto &mp: MPs) {
        auto pos = Converter::toCvMat(t12.map(Converter::toVector3d(mp->GetWorldPos())));
        pMap->AddUpdate(new MapPointUpdate<cv::Mat>(
                mp->mnId, "SetWorldPos", pos));
        // TODO(halcao): if mp is sent but set bad by the server, how to update in client?
        if (mp->mbToBeSerialized && mp->isBad()) {
            warn("mp {} is bad but will be sent back", mp->mnId);
        }
    }


    SerializeMap(pMap, result);
    UpdateMap(SLAM->GetMap(), result);

    auto newKFs = SLAM->GetMap()->GetAllKeyFrames();
    auto newMPs = SLAM->GetMap()->GetAllMapPoints();
    auto originalMapId = SLAM->GetMap()->mnId;
    for (auto &kf: newKFs) {
        if (kf && kf->GetOriginMapId() != originalMapId) {
            kf->mbToBeSerialized = false;
        }
    }

    for (auto &mp: newMPs) {
        if (mp && mp->mnId / MAP_BASE != originalMapId) {
            mp->mbToBeSerialized = false;
        }
    }
}

void CheckOverlap() {
    for (size_t i = 0; i < clients.size(); ++i) {
        globalClient->CheckOverlapCandidates(clients[i]);
    }
}

int main(int argc, char **argv) {
    if (argc < 6) {
        error("Usage: ./map_conveyance path_to_vocabulary path_to_settings path_to_image_folder path_to_times_file/tum debug_level [use_viewer] [use_map_viewer]");
        return 1;
    }

    // Retrieve paths to images
    vector<string> vstrImageFilenames;
    vector<double> vTimestamps;

    auto option4 = string(argv[4]);
    if (option4 == "tum") {
        DataSetUtil::LoadTUM(string(argv[3]), vstrImageFilenames, vTimestamps);
    } else {
        DataSetUtil::LoadEuRoC(string(argv[3]), option4, vstrImageFilenames, vTimestamps);
    }


    int nImages = vstrImageFilenames.size();

    if (nImages <= 0) {
        error("ERROR: Failed to load images");
        return 1;
    }

    CLogger::SetLevel(argv[5]);
//    Config::load(argv[5]);

    // 等于0就不用
    bool use_viewer = argc > 6 ?
                      argv[6][0] != '0' : true;


    bool use_viewer2 = argc > 7 ?
                       argv[7][0] != '0' : true;

    auto pVoc = new ORBVocabulary();
    pVoc->loadFromBinaryFile(argv[1]);
//    pkfDB = new KeyFrameDatabase(*pVoc);


    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    const int nClient = 2;
    for (int i = 0; i < nClient; ++i) {
        // KEY: the constructor will change argv[2] so that the gl and cv process can not be done
        string str1 = argv[2];
        string str2 = argv[2];
        auto client = new ORB_SLAM2::Client(str1, pVoc, use_viewer2);
        auto SLAM = new ORB_SLAM2::System(argv[1], str2, ORB_SLAM2::System::MONOCULAR, use_viewer);
        clients.push_back(client);
        SLAMs.push_back(SLAM);
    }

    auto pVoc2 = new ORBVocabulary();
    pVoc2->loadFromBinaryFile(argv[1]);

    string str0 = argv[2];
    globalClient = new ORB_SLAM2::Client(str0, pVoc2, use_viewer2);


    auto tRetriever = new thread([]() {
        b.store(true);
        while (b.load()) {
            std::this_thread::sleep_for(chrono::milliseconds(5000));
            for (size_t i = 0; i < SLAMs.size(); i++) {
                ReceiveMap(SLAMs[i], clients[i]);
//                ReceiveMap(SLAMs[i], globalClient);
            }
            CheckOverlap();
        }
    });

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    info("-------");
    info("Start processing sequence ...");
    info("Images in the sequence: {}", nImages);

    nImages /= nClient;

    int nOverlap = 15;

    // Main loop
    for (int ni = 0; ni < nImages + nOverlap; ni++) {
        Timer tTrack;

        vector<thread *> threads;
        // Pass the image to the SLAM system
        for (int i = 0; i < nClient; ++i) {
            auto idx = nImages * i + ni;
            if (size_t(idx) >= vstrImageFilenames.size()) continue;
            auto imageName = vstrImageFilenames[idx];
            // Read image from file
            double tframe = vTimestamps[idx];

            auto SLAM = SLAMs[i];
            auto ptTracking = new thread([SLAM, tframe, imageName]() {
                Timer t;
                cv::Mat im = cv::imread(imageName, CV_LOAD_IMAGE_UNCHANGED);
                if (im.empty()) {
                    error("Failed to load image at: {}", imageName);
                    return 1;
                }

                SLAM->TrackMonocular(im, tframe);
            });
            threads.push_back(ptTracking);
            if (!ptTracking) {
                warn("pt Tracking {} null", i);
            }
        }

        for (int i = 0; i < nClient; ++i) {
            // if overlap happened, skip the join
            if (i < threads.size() && threads[i] && threads[i]->joinable()) {
                threads[i]->join();
                delete threads[i];
            }
        }

        vTimesTrack[ni] = tTrack.get();
        trace("full track time: {}", vTimesTrack[ni]);

//            // Wait to load the next frame
//            double T = 0;
//            if (ni < nImages - 1)
//                T = vTimestamps[ni + 1] - tframe;
//            else if (ni > 0)
//                T = tframe - vTimestamps[ni - 1];

//        if(ttrack<T)
//            usleep((T-ttrack)*1e6);
    }

    if (use_viewer || use_viewer2) {
        info("press any key to stop");
        getchar();
    }

    // Stop all threads
    b.store(false);
    debug("wait tRetriever to stop");
    tRetriever->join();

    for (auto &SLAM: SLAMs) {
        SLAM->Shutdown();
    }

    for (auto &client: clients) {
        client->Shutdown();
    }

    if (globalClient) {
        globalClient->Shutdown();
    }

    // Tracking time statistics
    sort(vTimesTrack.begin(), vTimesTrack.end());
    float totaltime = 0;
    for (int ni = 0; ni < nImages; ni++) {
        totaltime += vTimesTrack[ni];
    }
    info("median tracking time: {}", vTimesTrack[nImages / 2]);
    info("mean tracking time: {}", totaltime / nImages);

    // Save camera trajectory
    for (size_t i = 0; i < SLAMs.size(); ++i) {
        auto SLAM = SLAMs[i];
        auto client = clients[i];
        debug("original map {} keyframe count: {}", SLAM->GetMap()->mnId, SLAM->GetMap()->KeyFramesInMap());
        debug("original map {} mappoint count: {}", SLAM->GetMap()->mnId, SLAM->GetMap()->MapPointsInMap());
        SLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory" + std::to_string(SLAM->GetMap()->mnId) + ".txt");
        SLAM->SaveMap("map-old-" + std::to_string(SLAM->GetMap()->mnId) + ".bin");

        debug("new map {} keyframe count: {}", client->GetMap()->mnId, client->GetMap()->KeyFramesInMap());
        debug("new map {} mappoint count: {}", client->GetMap()->mnId, client->GetMap()->MapPointsInMap());

        client->SaveMap("map-new-" + std::to_string(client->GetMap()->mnId) + ".bin");
    }

    MapManager::SaveGlobalMap("map");

    return 0;
}
