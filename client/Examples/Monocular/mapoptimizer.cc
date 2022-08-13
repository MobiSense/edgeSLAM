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


#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include <opencv2/core/core.hpp>
#include <BoostArchiver.h>
#include <nanoflann.hpp>

#include <Map.h>
#include <Viewer.h>
#include <KeyFrameDatabase.h>
#include <ORBmatcher.h>
#include <PnPsolver.h>
#include <Optimizer.h>
#include <ORBextractor.h>
#include <Converter.h>

using namespace std;
using namespace ORB_SLAM2;
using namespace nanoflann;

template<typename T>
struct PointCloud {
    struct Point {
        T x, y, z;
    };

    std::vector<Point> pts;

    // Must return the number of data points
    inline size_t kdtree_get_point_count() const { return pts.size(); }

    // Returns the dim'th component of the idx'th point in the class:
    // Since this is inlined and the "dim" argument is typically an immediate value, the
    //  "if/else's" are actually solved at compile time.
    inline T kdtree_get_pt(const size_t idx, const size_t dim) const {
        if (dim == 0) return pts[idx].x;
        else if (dim == 1) return pts[idx].y;
        else return pts[idx].z;
    }

    // Optional bounding-box computation: return false to default to a standard bbox computation loop.
    //   Return true if the BBOX was already computed by the class and returned in "bb" so it can be avoided to redo it again.
    //   Look at bb.size() to find out the expected dimensionality (e.g. 2 or 3 for point clouds)
    template<class BBOX>
    bool kdtree_get_bbox(BBOX & /* bb */) const { return false; }

};


void loadMap(const string &filename, Map *&pMap, KeyFrameDatabase *&pKeyFrameDatabase, ORBVocabulary *&pVoc) {
    std::ifstream in(filename, std::ios_base::binary);
    if (!in) {
        cerr << "Cannot Open Mapfile: " << filename << " , Create a new one" << std::endl;
        return;
    }
    cout << "Loading Mapfile: " << filename << std::flush;
    boost::archive::binary_iarchive ia(in, boost::archive::no_header);
    ia >> pMap;
    ia >> pKeyFrameDatabase;
    in.close();

    cout << " ...done" << std::endl;
    cout << "Map Reconstructing" << flush;

    pKeyFrameDatabase->SetORBvocabulary(pVoc);
    vector<ORB_SLAM2::KeyFrame *> vpKFS = pMap->GetAllKeyFrames();
    for (auto it:vpKFS) {
        it->mpKeyFrameDB = pKeyFrameDatabase;
        it->ComputeBoW();
    }
    cout << " ...done" << endl;
}


//Calibration matrix
cv::Mat mK;
cv::Mat mDistCoef;
float mbf;
//Color order (true RGB, false BGR, ignored if grayscale)
bool mbRGB;
ORBextractor *pORBextractor;

void loadSettings(const string &strSettingPath) {
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];

    cv::Mat K = cv::Mat::eye(3, 3, CV_32F);
    K.at<float>(0, 0) = fx;
    K.at<float>(1, 1) = fy;
    K.at<float>(0, 2) = cx;
    K.at<float>(1, 2) = cy;
    K.copyTo(mK);

    cv::Mat DistCoef(4, 1, CV_32F);
    DistCoef.at<float>(0) = fSettings["Camera.k1"];
    DistCoef.at<float>(1) = fSettings["Camera.k2"];
    DistCoef.at<float>(2) = fSettings["Camera.p1"];
    DistCoef.at<float>(3) = fSettings["Camera.p2"];
    const float k3 = fSettings["Camera.k3"];
    if (k3 != 0) {
        DistCoef.resize(5);
        DistCoef.at<float>(4) = k3;
    }
    DistCoef.copyTo(mDistCoef);

    mbf = fSettings["Camera.bf"];

    cout << endl << "Camera Parameters: " << endl;
    cout << "- fx: " << fx << endl;
    cout << "- fy: " << fy << endl;
    cout << "- cx: " << cx << endl;
    cout << "- cy: " << cy << endl;
    cout << "- k1: " << DistCoef.at<float>(0) << endl;
    cout << "- k2: " << DistCoef.at<float>(1) << endl;
    if (DistCoef.rows == 5)
        cout << "- k3: " << DistCoef.at<float>(4) << endl;
    cout << "- p1: " << DistCoef.at<float>(2) << endl;
    cout << "- p2: " << DistCoef.at<float>(3) << endl;
//    cout << "- fps: " << fps << endl;


    int nRGB = fSettings["Camera.RGB"];
    mbRGB = nRGB;

    if (mbRGB)
        cout << "- color order: RGB (ignored if grayscale)" << endl;
    else
        cout << "- color order: BGR (ignored if grayscale)" << endl;

    // Load ORB parameters

    int nFeatures = fSettings["ORBextractor.nFeatures"];
    float fScaleFactor = fSettings["ORBextractor.scaleFactor"];
    int nLevels = fSettings["ORBextractor.nLevels"];
    int fIniThFAST = fSettings["ORBextractor.iniThFAST"];
    int fMinThFAST = fSettings["ORBextractor.minThFAST"];

    pORBextractor = new ORBextractor(nFeatures, fScaleFactor, nLevels, fIniThFAST, fMinThFAST);

}

bool Relocalization(Frame &currentFrame, KeyFrameDatabase *&pKeyFrameDB) {
    // Compute Bag of Words Vector
    currentFrame.ComputeBoW();

    // Relocalization is performed when tracking is lost
    // Track Lost: Query KeyFrame Database for keyframe candidates for relocalisation
    vector<KeyFrame *> vpCandidateKFs = pKeyFrameDB->DetectRelocalizationCandidates(&currentFrame);

    if (vpCandidateKFs.empty())
        return false;

    const int nKFs = vpCandidateKFs.size();

    // We perform first an ORB matching with each candidate
    // If enough matches are found we setup a PnP solver
    ORBmatcher matcher(0.75, true);

    vector<PnPsolver *> vpPnPsolvers;
    vpPnPsolvers.resize(nKFs);

    vector<vector<MapPoint *> > vvpMapPointMatches;
    vvpMapPointMatches.resize(nKFs);

    vector<bool> vbDiscarded;
    vbDiscarded.resize(nKFs);

    int nCandidates = 0;

    for (int i = 0; i < nKFs; i++) {
        KeyFrame *pKF = vpCandidateKFs[i];
        if (pKF->isBad())
            vbDiscarded[i] = true;
        else {
            int nmatches = matcher.SearchByBoW(pKF, currentFrame, vvpMapPointMatches[i]);
            if (nmatches < 15) {
                vbDiscarded[i] = true;
                continue;
            } else {
                PnPsolver *pSolver = new PnPsolver(currentFrame, vvpMapPointMatches[i]);
                pSolver->SetRansacParameters(0.99, 10, 300, 4, 0.5, 5.991);
                vpPnPsolvers[i] = pSolver;
                nCandidates++;
            }
        }
    }

    // Alternatively perform some iterations of P4P RANSAC
    // Until we found a camera pose supported by enough inliers
    bool bMatch = false;
    ORBmatcher matcher2(0.9, true);

    while (nCandidates > 0 && !bMatch) {
        for (int i = 0; i < nKFs; i++) {
            if (vbDiscarded[i])
                continue;

            // Perform 5 Ransac Iterations
            vector<bool> vbInliers;
            int nInliers;
            bool bNoMore;

            PnPsolver *pSolver = vpPnPsolvers[i];
            cv::Mat Tcw = pSolver->iterate(5, bNoMore, vbInliers, nInliers);

            // If Ransac reachs max. iterations discard keyframe
            if (bNoMore) {
                vbDiscarded[i] = true;
                nCandidates--;
            }

            // If a Camera Pose is computed, optimize
            if (!Tcw.empty()) {
//                Tcw.copyTo(currentFrame.mTcw);
                currentFrame.SetPose(Tcw);

                set<MapPoint *> sFound;

                const int np = vbInliers.size();

                for (int j = 0; j < np; j++) {
                    if (vbInliers[j]) {
                        currentFrame.mvpMapPoints[j] = vvpMapPointMatches[i][j];
                        sFound.insert(vvpMapPointMatches[i][j]);
                    } else
                        currentFrame.mvpMapPoints[j] = NULL;
                }

                int nGood = Optimizer::PoseOptimization(&currentFrame);

                if (nGood < 10)
                    continue;

                for (int io = 0; io < currentFrame.N; io++)
                    if (currentFrame.mvbOutlier[io])
                        currentFrame.mvpMapPoints[io] = static_cast<MapPoint *>(NULL);

                // If few inliers, search by projection in a coarse window and optimize again
                if (nGood < 50) {
                    int nadditional = matcher2.SearchByProjection(currentFrame, vpCandidateKFs[i], sFound, 10, 100);

                    if (nadditional + nGood >= 50) {
                        nGood = Optimizer::PoseOptimization(&currentFrame);

                        // If many inliers but still not enough, search by projection again in a narrower window
                        // the camera has been already optimized with many points
                        if (nGood > 30 && nGood < 50) {
                            sFound.clear();
                            for (int ip = 0; ip < currentFrame.N; ip++)
                                if (currentFrame.mvpMapPoints[ip])
                                    sFound.insert(currentFrame.mvpMapPoints[ip]);
                            nadditional = matcher2.SearchByProjection(currentFrame, vpCandidateKFs[i], sFound, 3, 64);

                            // Final optimization
                            if (nGood + nadditional >= 50) {
                                nGood = Optimizer::PoseOptimization(&currentFrame);

                                for (int io = 0; io < currentFrame.N; io++)
                                    if (currentFrame.mvbOutlier[io])
                                        currentFrame.mvpMapPoints[io] = NULL;
                            }
                        }
                    }
                }


                // If the pose is supported by enough inliers stop ransacs and continue
                if (nGood >= 50) {
                    bMatch = true;
                    break;
                }
            }
        }
    }

    if (!bMatch) {
        return false;
    } else {
//        mnLastRelocFrameId = currentFrame.mnId;
        return true;
    }

}

void LoadImages(const string &strImagePath, const string &strPathTimes,
                vector<string> &vstrImages, vector<double> &vTimeStamps) {
    ifstream fTimes;
    fTimes.open(strPathTimes.c_str());
    vTimeStamps.reserve(5000);
    vstrImages.reserve(5000);
    while (!fTimes.eof()) {
        string s;
        getline(fTimes, s);
        if (!s.empty()) {
            stringstream ss;
            ss << s;
            vstrImages.push_back(strImagePath + "/" + ss.str() + ".png");
            double t;
            ss >> t;
            vTimeStamps.push_back(t / 1e9);

        }
    }
}

void LoadImagesTUM(const string &strImagePath, const string &strPathTimes, vector<string> &vstrImageFilenames,
                   vector<double> &vTimestamps) {
    ifstream f;
    f.open(strPathTimes.c_str());

    // skip first three lines
    string s0;
    getline(f, s0);
    getline(f, s0);
    getline(f, s0);

    while (!f.eof()) {
        string s;
        getline(f, s);
        if (!s.empty()) {
            stringstream ss;
            ss << s;
            double t;
            string sRGB;
            ss >> t;
            vTimestamps.push_back(t);
            ss >> sRGB;
            vstrImageFilenames.push_back(strImagePath + "/" + sRGB);
        }
    }
}

void SaveKeyFrameTrajectoryTUM(vector<Frame> &frames, const string &filename) {
    cout << endl << "Saving keyframe trajectory to " << filename << " ..." << endl;

    sort(frames.begin(), frames.end(), [](const Frame &f1, const Frame &f2) {
        return f1.mnId < f2.mnId;
    });

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    //cv::Mat Two = vpKFs[0]->GetPoseInverse();


    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    for (auto frame : frames) {
        // pKF->SetPose(pKF->GetPose()*Two);

        if (!frame.mTcw.data)
            continue;

        cv::Mat R = frame.GetRotationInverse();
        vector<float> q = Converter::toQuaternion(R);
        cv::Mat t = frame.GetCameraCenter();
        f << setprecision(6) << frame.mTimeStamp << setprecision(7) << " " << t.at<float>(0) << " " << t.at<float>(1)
          << " " << t.at<float>(2)
          << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;
    }

    f.close();
    cout << endl << "trajectory saved!" << endl;
}


void saveMap(const string &filename, Map *&mpMap, KeyFrameDatabase *&mpKeyFrameDatabase) {
    std::ofstream out(filename, std::ios_base::binary);
    if (!out) {
        cerr << "Cannot Write to Mapfile: " << filename << std::endl;
        exit(-1);
    }
    cout << "Saving Mapfile: " << filename << std::flush;
    boost::archive::binary_oarchive oa(out, boost::archive::no_header);
    oa << mpMap;
    oa << mpKeyFrameDatabase;
    cout << " ...done" << std::endl;

    cout << mpMap->KeyFramesInMap() << endl;
    out.close();
}

struct node {
    int i;
    int j;
    float key;

    node(int i, int j, float key) : i(i), j(j), key(key) {}
};

struct cmp {
    bool operator()(node a, node b) {
        return a.key > b.key;
    }
};

set<KeyFrame *> allKFs;
set<MapPoint *> allMPs;

void addMapPoint(MapPoint *mp);

void addKeyFrame(KeyFrame *kf) {
    if (!kf) return;
    if (allKFs.count(kf)) return;
    allKFs.insert(kf);

    for (auto &itr: kf->mvpMapPoints) {
        addMapPoint(itr);
    }

    for (auto &itr: kf->mConnectedKeyFrameWeights) {
        addKeyFrame(itr.first);
    }

    for (auto &itr: kf->mvpOrderedConnectedKeyFrames) {
        addKeyFrame(itr);
    }

    addKeyFrame(kf->mpParent);

    for (auto &itr: kf->mspChildrens) {
        addKeyFrame(itr);
    }

    for (auto &itr: kf->mspLoopEdges) {
        addKeyFrame(itr);
    }
}

void addMapPoint(MapPoint *mp) {
    if (!mp) return;
    if (allMPs.count(mp)) return;
    allMPs.insert(mp);

    for (auto &itr: mp->mObservations) {
        addKeyFrame(itr.first);
    }

    addKeyFrame(mp->mpRefKF);

    addMapPoint(mp->mpReplaced);
}

void startCount(Map *&map, KeyFrameDatabase *&kfdb) {
//    ar & map.mspMapPoints;
//    ar & map.mvpKeyFrameOrigins;
//    ar & map.mspKeyFrames;
//    ar & map.mvpReferenceMapPoints;
    for (auto &itr: map->mspMapPoints) {
        addMapPoint(itr);
    }

    for (auto &itr: map->mvpKeyFrameOrigins) {
        addKeyFrame(itr);
    }

    for (auto &itr: map->mspKeyFrames) {
        addKeyFrame(itr);
    }

    for (auto &itr: map->mvpReferenceMapPoints) {
        addMapPoint(itr);
    }

    for (auto &itr: kfdb->mvInvertedFile) {
        for (auto &it: itr) {
            addKeyFrame(it);
        }
    }
}

void deleteAllKeyframes(vector<unsigned int> deleted) {
    for (auto id: deleted) {
        for (auto &mp: allMPs) {
            //    mapPoint.mObservations;
            //    mapPoint.mpRefKF;
            //    mapPoint.mpReplaced;
            for (auto &itr: mp->mObservations) {
                if (itr.first->mnId == id) {
                    mp->EraseObservation(itr.first);
//                    mp->mObservations.erase(itr.first);
                }
            }

            if (mp->mpRefKF && mp->mpRefKF->mnId == id) {
                mp->mpRefKF = nullptr;
            }

        }

        for (auto &kf: allKFs) {
            //     keyframe.mvpMapPoints
            //     keyframe.mConnectedKeyFrameWeights
            //     keyframe.mvpOrderedConnectedKeyFrames
            //     keyframe.mpParent & keyframe.mspChildrens
            //     keyframe.mspLoopEdges
            if (kf->mnId == id) {
                kf->SetBadFlag();
            }

            for (auto &itr: kf->mConnectedKeyFrameWeights) {
                if (!itr.first) continue;
                if (itr.first->mnId == id) {
//                    kf->EraseConnection(itr.first);
                    kf->mConnectedKeyFrameWeights.erase(itr.first);
                }
            }

            for (auto &itr: kf->mvpOrderedConnectedKeyFrames) {
                if (itr->mnId == id) {
                    kf->mvpOrderedConnectedKeyFrames.erase(
                            std::remove(
                                    kf->mvpOrderedConnectedKeyFrames.begin(),
                                    kf->mvpOrderedConnectedKeyFrames.end(),
                                    itr),
                            kf->mvpOrderedConnectedKeyFrames.end());
                }
            }

            if (kf->mpParent && kf->mpParent->mnId == id) {
                kf->mpParent = nullptr;
            }

            for (auto &itr: kf->mspChildrens) {
                if (itr->mnId == id) {
                    kf->EraseChild(itr);
//                    kf->mspChildrens.erase(itr);
                }
            }

            for (auto &itr: kf->mspLoopEdges) {
                if (itr->mnId == id) {
                    kf->mspLoopEdges.erase(itr);
                }
            }
        }
    }

    for (auto &kf: allKFs) {
        kf->UpdateBestCovisibles();
    }
}

vector<unsigned int> method1(Map *&pMap) {
    vector<KeyFrame *> keyframes = pMap->GetAllKeyFrames();
    size_t total = keyframes.size();

    PointCloud<float> cloud;
    cloud.pts.resize(total);

    for (size_t i = 0; i < total; ++i) {
        auto kf = keyframes[i];
        cv::Mat Twc = kf->GetPoseInverse().t();
        float x = Twc.at<float>(3, 0);
        float y = Twc.at<float>(3, 1);
        float z = Twc.at<float>(3, 2);
        cloud.pts[i] = {x, y, z};
    }

    float query_pt[3] = {0.5, 0.5, 0.5};

    // construct a kd-tree index:
    typedef KDTreeSingleIndexAdaptor<
            L2_Simple_Adaptor<float, PointCloud<float> >,
            PointCloud<float>,
            3 /* dim */
    > my_kd_tree_t;

    my_kd_tree_t index(3, cloud, KDTreeSingleIndexAdaptorParams(10 /* max leaf */));
    index.buildIndex();
    // 思路 1. 统计i到j的距离，建立矩阵，找到分布情况，去除距离小于50%的
    priority_queue<node, vector<node>, cmp> queue;
    for (int i = 0; i < total; ++i) {
        for (int j = i + 1; j < total; ++j) {
            auto pi = cloud.pts[i];
            auto pj = cloud.pts[j];
            float dx = pi.x - pj.x;
            float dy = pi.y - pj.y;
            float dz = pi.z - pj.z;

            float distance = sqrt(dx * dx + dy * dy + dz * dz);
            queue.emplace(i, j, distance);
        }
    }

    int size = queue.size();
    cout << size << endl;

    unordered_map<int, vector<int> > map;
    for (int i = 0; i < size; ++i) {
        node n = queue.top();
        queue.pop();
        if (i < size / 2) {
            if (map.find(n.i) == map.end()) {
                map[n.i] = vector<int>();
            }
            map[n.i].push_back(n.j);
        }

        cout << "i: " << n.i << " j: " << n.j << " dist: " << n.key << endl;
    }


    std::vector<std::pair<int, vector<int>>> elems(map.begin(), map.end());
    sort(elems.begin(), elems.end(), [](std::pair<int, vector<int>> a, std::pair<int, vector<int>> b) {
        return a.second.size() < b.second.size();
    });

    vector<unsigned int> vDeletedKF;
    for (int i = 0; i < elems.size(); ++i) {
        int neighborSize = elems[i].second.size();
        if (neighborSize > 10) {
            auto kf = keyframes[elems[i].first];
            kf->SetBadFlag();
            vDeletedKF.push_back(elems[i].first);
        }
    }

    return vDeletedKF;
}

void optimizeMap(Map *&pMap, KeyFrameDatabase *&pKeyFrameDB, int threshold) {
    startCount(pMap, pKeyFrameDB);
    cout << "Keyframe Count: " << allKFs.size() << endl;
    cout << "MapPoint Count: " << allMPs.size() << endl;

    auto total_count = pMap->KeyFramesInMap();

    // 思路 1. 统计i到j的距离，建立矩阵，找到分布情况，去除距离小于50%的

//    auto deleted = method1(pMap);
//    deleteAllKeyframes(deleted);

    // 思路 2. 进行聚类，去除每个聚类中50%的


    // 思路 3. 使得共视的点数在阈值之内(删除共视较多的点？)
    // 问题描述，建立起一个covisibility map

    auto cmp = [](pair<size_t, double> a, pair<size_t, double> b){
        return a.second < b.second;
    };

    priority_queue<pair<size_t, double>, vector<pair<size_t, double> >, decltype(cmp)> queue(cmp);

    map<pair<unsigned, unsigned>, unsigned> matrix;
    auto keyframes = pMap->GetAllKeyFrames();


    // 删掉不在map里的
    vector<unsigned> deleted;
    for (auto &kf: allKFs) {
        if (find(keyframes.begin(), keyframes.end(), kf) == keyframes.end()) {
            deleted.push_back(kf->mnId);
        }
    }
//    deleteAllKeyframes(deleted);

    cout << "removed / total = " << deleted.size()  << " / " << allKFs.size() << endl;


    for (size_t i = 0; i < keyframes.size(); i++) {
        auto kf1 = keyframes[i];
        for (size_t j = i+1; j < keyframes.size(); j++) {
            auto kf2 = keyframes[j];
            if (kf1->mnId == kf2->mnId) continue;

            set<unsigned> points1;

            for (auto &mp: kf1->mvpMapPoints) {
                if (!mp) continue;
                points1.insert(mp->mnId);
            }

            set<unsigned> points2;
            for (auto &mp: kf2->mvpMapPoints) {
                if (!mp) continue;
                points2.insert(mp->mnId);
            }
            vector<unsigned> intersect;
            set_intersection(points1.begin(), points1.end(),
                    points2.begin(), points2.end(), back_inserter(intersect));

            double covisibility = double(intersect.size()) / double(max(points1.size(), points2.size()));
            matrix[make_pair(kf1->mnId, kf2->mnId)] = covisibility;
            cout << kf1->mnId << ", " << kf2->mnId << ": " << covisibility << endl;
            queue.push(make_pair(i, covisibility));
        }
    }

    deleted.clear();
    for (size_t i = 0; i < keyframes.size() * 0.5; i++) {
        auto itr = queue.top();
        queue.pop();
        cout << "index: " << itr.first << " mnId: " << keyframes[itr.first]->mnId << " covisibility: " << itr.second << endl;
        deleted.push_back(keyframes[itr.first]->mnId);
        if (keyframes[itr.first]->mbNotErase) {
            cout << " mnId: " << keyframes[itr.first]->mnId << "mbNotErase" << endl;
        }
//        keyframes[itr.first]->SetBadFlag();
    }

    deleteAllKeyframes(deleted);

    auto now_count = pMap->KeyFramesInMap();
    cout << "removed / total = " << total_count - now_count << " / " << total_count << endl;
}

int main(int argc, char **argv) {
    if (argc != 5) {
        cerr << endl << "Usage: ./optimizer map_file_path setting_file_path voc_path mp_threshold" << endl;
        return 1;
    }


    const string filename = argv[1];
    auto pMap = new Map();
    //Load ORB Vocabulary
    cout << endl << "Loading ORB Vocabulary. This could take a while..." << endl;

    const string vocPath = argv[3];

    auto pVocabulary = new ORBVocabulary();
    bool bVocLoad = pVocabulary->loadFromBinaryFile(vocPath);
    if (!bVocLoad) {
        cerr << "Wrong path to vocabulary. " << endl;
        cerr << "Failed to open at: " << vocPath << endl;
        exit(-1);
    }

    auto pkfDB = new KeyFrameDatabase(*pVocabulary);

    // load map
    std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
    loadMap(filename, pMap, pkfDB, pVocabulary);
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    double ttrack = std::chrono::duration_cast<std::chrono::duration<double> >(end - start).count();
    cout << "map load time: " << ttrack << endl;

    vector<string> filenames;
    vector<double> timestamps;
    vector<bool> results;
    vector<Frame> frames;
    cout << "loading image\n";

    int threshold = stoi(argv[4]);
    optimizeMap(pMap, pkfDB, threshold);
    saveMap("mh01_new.bin", pMap, pkfDB);


    // initialize map & frame drawer
    auto pFrameDrawer = new FrameDrawer(pMap);
    auto pMapDrawer = new MapDrawer(pMap, argv[2]);
    loadSettings(argv[2]);
    auto pViewer = new Viewer(nullptr, pFrameDrawer, pMapDrawer, nullptr, argv[2]);
    auto tViewer = thread([&pViewer]() {
        pViewer->Run();
    });
    cv::waitKey();
    return 0;
}

