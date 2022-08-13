//
// Created by Halcao on 2020/5/14.
//

#ifndef EDGE_SLAM_CLIENT_H
#define EDGE_SLAM_CLIENT_H

#include <string>
#include <thread>

namespace ORB_SLAM2 {

using std::string;
class Viewer;
class FrameDrawer;
class MapDrawer;
class Map;
class LoopClosing;
class KeyFrameDatabase;

class Client {
public:
    typedef pair<set<KeyFrame*>,int> ConsistentGroup;

    // Initialize the SLAM system. It launches the Local Mapping, Loop Closing and Viewer threads.
    Client(const string &strSettingsFile, ORBVocabulary *pVoc, const bool bUseViewer = true);

    // All threads will be requested to finish.
    // It waits until all threads have finished.
    // This function must be called before saving the trajectory.
    void Shutdown();

    void SaveMap(const string &filename);
    void CheckOverlapCandidates(const Client *other);
    inline Map *GetMap() {
        return mpMap;
    }

    id_t mnId;
private:
    static id_t nNextId;
    // SLAM Map
    Map* mpMap;
    // keyframes from other client
    map<Client *, unsigned long> sentKeyFrames;
    // mappoints from other client
    map<Client *, unsigned long> sentMapPoints;

    // Loop Closer. It searches loops with every new keyframe. If there is a loop it performs
    // a pose graph optimization and full bundle adjustment (in a new thread) afterwards.
    LoopClosing* mpLoopCloser;

    // The viewer draws the map and the current camera pose. It uses Pangolin.
    FrameDrawer* mpFrameDrawer;
    MapDrawer* mpMapDrawer;
    Viewer* mpViewer;
    KeyFrameDatabase* mpKeyFrameDatabase;

    // threads
    std::thread* mptLoopClosing;
    std::thread* mptViewer = nullptr;

    // last checked ids
    std::map<unsigned long, unsigned long> lastCheckedKFIds;
    std::map<unsigned long, unsigned long> lastCheckedMPIds;

    // used for loop detection
    std::map<unsigned long, std::vector<ConsistentGroup> > mvConsistentGroupsMap;
    std::map<unsigned long, std::vector<KeyFrame*> > mvpEnoughConsistentCandidatesMap;

    void GetSim3(KeyFrame *pCurrentKF, vector<KeyFrame *> candidates);

    bool DetectLoop(KeyFrame *mpCurrentKF, float minScore, vector<KeyFrame *> vpCandidateKFs);

    void RunGlobalBundleAdjustment(Map *pMap1, Map* pMap2);

};
};

#endif //EDGE_SLAM_CLIENT_H
