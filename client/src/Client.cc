//
// Created by Halcao on 2020/5/14.
//

#include <ORBVocabulary.h>
#include <boost/archive/binary_oarchive.hpp>

#include "Optimizer.h"
#include "LoopClosing.h"
#include "KeyFrameDatabase.h"
#include "Client.h"
#include "MapUpdater.h"
#include "BoostArchiver.h"
#include "ORBmatcher.h"
#include "PnPsolver.h"
#include "Sim3Solver.h"
#include "Converter.h"
#include "Optimizer.h"
#include "MapManager.h"
#include <CLogger.h>

namespace ORB_SLAM2 {
    id_t Client::nNextId = 0;

    Client::Client(const string &strSettingsFile, ORBVocabulary *pVoc, const bool bUseViewer) :
            mnId(nNextId++) {
        mpKeyFrameDatabase = new KeyFrameDatabase(*pVoc);

        mpMap = new Map();

        MapManager::Register(mpMap);

        // monocular so fixedScale = false
        mpLoopCloser = new LoopClosing(mpMap, mpKeyFrameDatabase, pVoc, false);
        mptLoopClosing = new thread(&LoopClosing::Run, mpLoopCloser);

        mpFrameDrawer = new FrameDrawer(mpMap);
        mpMapDrawer = new MapDrawer(mpMap, strSettingsFile);
        // system and tracking null
        mpViewer = new Viewer(nullptr, mpFrameDrawer, mpMapDrawer, nullptr, strSettingsFile);
        if (bUseViewer) {
            auto id = to_string(mpMap->mnId);
            mpViewer->SetTitle("Server Map Viewer " + id, "Server Frame Viewer " + id);

            mptViewer = new thread(&Viewer::Run, mpViewer);
        }
    }

    void Client::Shutdown() {
        debug("wait pLoopCloser to finish");
        mpLoopCloser->RequestFinish();
        // Wait until all thread have effectively stopped
        while (!mpLoopCloser->isFinished()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        if (mptLoopClosing) mptLoopClosing->join();

        if (mptViewer) {
            info("wait pViewer to finish");
            mpViewer->RequestFinish();
            while (!mpViewer->isFinished()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
            }
            mptViewer->join();
        }
    }

    void Client::SaveMap(const string &filename) {
        std::ofstream out(filename, std::ios_base::binary);
        if (!out) {
            error("Cannot Write to Mapfile: {}", filename);
            exit(-1);
        }
        info("Saving Mapfile: {}", filename);
        boost::archive::binary_oarchive oa(out, boost::archive::no_header);
        for (auto &itr: mpMap->GetKeyFrameMaps()) {
            itr.second->SetupSerializationVariable();
            for (auto i = 0; i < itr.second->mvKeysUn.size(); ++i) {
                auto kp = itr.second->mvKeysUn[i];
                if (kp.angle > 360 || kp.angle < 0) {
                    warn("key point {} of keyframe {} is {}", i, itr.second->mnId, kp.angle);
                }
            }
        }

        for (auto &itr: mpMap->GetMapPointMaps()) {
            itr.second->SetupSerializationVariable();
        }
        mpKeyFrameDatabase->SetupSerializationVariable();

        oa << mpMap;
        oa << mpKeyFrameDatabase;
        info("...done");
        out.close();
    }

    void Client::CheckOverlapCandidates(const Client* other) {
        auto vpKFs = other->mpMap->allKFs;
        auto vpMPs = other->mpMap->allMPs;
        // init variables
        if (lastCheckedKFIds.count(other->mpMap->mnId) == 0) lastCheckedKFIds[other->mpMap->mnId] = 0;
        if (lastCheckedMPIds.count(other->mpMap->mnId) == 0) lastCheckedMPIds[other->mpMap->mnId] = 0;

        // Get last tracked id
        auto lastCheckedKFId = lastCheckedKFIds[other->mpMap->mnId];
        auto lastCheckedMPId = lastCheckedMPIds[other->mpMap->mnId];

        for (auto &itr: vpMPs) {
            auto mp = itr.second;
            // if tracked, ignore it
            if (!mp || mp->mnId <= lastCheckedMPId) continue;
            lastCheckedMPId = max(lastCheckedMPId, mp->mnId);

            if (!mp->isBad()) {
                mpMap->AddMapPoint(mp);
            }
        }
        lastCheckedMPIds[other->mpMap->mnId] = lastCheckedMPId;

        for (auto &itr: vpKFs) {
            auto kf = itr.second;
            // if tracked, ignore it
            if (!kf || kf->mnId <= lastCheckedKFId) continue;
            lastCheckedKFId = max(lastCheckedKFId, kf->mnId);

            if (!kf->isBad()) {
                mpMap->AddKeyFrame(kf);
            }

            auto minScore = kf->GetMinCovisibilityScore();
            auto candidates = mpKeyFrameDatabase->DetectLoopCandidates(kf, minScore);

            for (auto it = candidates.begin(); it != candidates.end(); ) {
                if ((*it)->mpMap->mnId == kf->mpMap->mnId) {
                    it = candidates.erase(it);
                } else {
                    ++it;
                }
            }
            debug("Global check map {} kf {} candidates count: {}", kf->mpMap->mnId, kf->mnId, candidates.size());


            if (DetectLoop(kf, minScore, candidates)) {
                GetSim3(kf, mvpEnoughConsistentCandidatesMap[kf->mpMap->mnId]);
            }
            mpKeyFrameDatabase->add(kf);
        }

        lastCheckedKFIds[other->mpMap->mnId] = lastCheckedKFId;
    }

    void Client::GetSim3(KeyFrame *pCurrentKF, vector<KeyFrame *> candidates) {
        const int nInitialCandidates = candidates.size();
        const bool mbFixScale = false;

        // We compute first ORB matches for each candidate
        // If enough matches are found, we setup a Sim3Solver
        ORBmatcher matcher(0.75, true);

        vector<Sim3Solver *> vpSim3Solvers;
        vpSim3Solvers.resize(nInitialCandidates);

        vector<vector<MapPoint *> > vvpMapPointMatches;
        vvpMapPointMatches.resize(nInitialCandidates);

        vector<bool> vbDiscarded;
        vbDiscarded.resize(nInitialCandidates);

        int nCandidates = 0; //candidates with enough matches

        for (int i = 0; i < nInitialCandidates; i++) {
            KeyFrame *pKF = candidates[i];

            // avoid that local mapping erase it while it is being processed in this thread
            pKF->SetNotErase();

            if (pKF->isBad()) {
                vbDiscarded[i] = true;
                continue;
            }

            int nmatches = 0;

            // if pKF is base map and pCurrentKF's map is merged
//            if (pKF->mpMap->mbBaseMap && pCurrentKF->mpMap->mbMerged) {
//                vvpMapPointMatches[i] = vector<MapPoint*>(pCurrentKF->GetMapPointMatches().size(), nullptr);
//                auto vpSrcMPs = pKF->GetMapPointMatches();
//                auto vpMapPoints = vector<MapPoint *>();
//                std::copy_if(vpSrcMPs.begin(), vpSrcMPs.end(), std::back_inserter(vpMapPoints), [](MapPoint *pMP){ return pMP != nullptr; });
//                nmatches = matcher.SearchByProjection(pCurrentKF, pCurrentKF->GetGlobalPose(), vpMapPoints, vvpMapPointMatches[i], 10);
//            } else {
                nmatches = matcher.SearchByBoW(pCurrentKF, pKF, vvpMapPointMatches[i]);
//            }


            info("kf {} and {} has {} matches", pCurrentKF->mnId, pKF->mnId, nmatches);

            if (nmatches < 20) {
                vbDiscarded[i] = true;
                continue;
            } else {
                Sim3Solver *pSolver = new Sim3Solver(pCurrentKF, pKF, vvpMapPointMatches[i], mbFixScale);
                pSolver->SetRansacParameters(0.99, 20, 300);
                vpSim3Solvers[i] = pSolver;
            }

            nCandidates++;
        }

//        if (nCandidates > 0) {
        debug("candidate total count: {}, nCandidates: {} with enough matches", nInitialCandidates, nCandidates);
//        }

        bool bMatch = false;

        // Perform alternatively RANSAC iterations for each candidate
        // until one is successful or all fail
        while (nCandidates > 0 && !bMatch) {
            for (int i = 0; i < nInitialCandidates; i++) {
                if (vbDiscarded[i])
                    continue;

                KeyFrame *pKF = candidates[i];

                // Perform 5 Ransac Iterations
                vector<bool> vbInliers;
                int nInliers;
                bool bNoMore;

                Sim3Solver *pSolver = vpSim3Solvers[i];
                cv::Mat Scm = pSolver->iterate(5, bNoMore, vbInliers, nInliers);

                // If Ransac reachs max. iterations discard keyframe
                if (bNoMore) {
                    vbDiscarded[i] = true;
                    nCandidates--;
                }

                // If RANSAC returns a Sim3, perform a guided matching and optimize with all correspondences
                if (!Scm.empty()) {
                    debug("RANSAC Get a Sim3");
                    vector<MapPoint *> vpMapPointMatches(vvpMapPointMatches[i].size(), static_cast<MapPoint *>(NULL));
                    for (size_t j = 0, jend = vbInliers.size(); j < jend; j++) {
                        if (vbInliers[j])
                            vpMapPointMatches[j] = vvpMapPointMatches[i][j];
                    }

                    cv::Mat R = pSolver->GetEstimatedRotation();
                    cv::Mat t = pSolver->GetEstimatedTranslation();
                    const float s = pSolver->GetEstimatedScale();
                    matcher.SearchBySim3(pCurrentKF, pKF, vpMapPointMatches, s, R, t, 7.5);
//
                    // gScm transform pKF to pCurrentKF
                    g2o::Sim3 gScm(Converter::toMatrix3d(R), Converter::toVector3d(t), s);
                    const int nInliers = Optimizer::OptimizeSim3(pCurrentKF, pKF, vpMapPointMatches, gScm, 10,
                                                                 mbFixScale);

                    // If optimization is successful stop RANSACs and continue
                    debug("nInliers {}", nInliers);
                    if (nInliers >= 40) {
//                    if (nInliers >= 20) {
                        bMatch = true;

                        g2o::Sim3 T12 = g2o::Sim3(Converter::toMatrix3d(pCurrentKF->GetRotation()),
                                                           Converter::toVector3d(pCurrentKF->GetTranslation()),
                                                           1.0).inverse() * gScm * g2o::Sim3(Converter::toMatrix3d(pKF->GetRotation()),
                                                         Converter::toVector3d(pKF->GetTranslation()), 1.0);
                        cout << "Merge map Sim3 scale " << T12 << endl;

                        info("map {} kf {} and map {} kf {} has a sim3", pCurrentKF->mpMap->mnId, pCurrentKF->mnId,
                             pKF->mpMap->mnId, pKF->mnId);
                        info("Start to Merge Map");
                        MapManager::MergeMap(pCurrentKF->mpMap, pKF->mpMap, T12);

                        break;
                    }
                }
            }
        }

        if (!bMatch) return;
    }

    bool Client::DetectLoop(KeyFrame *mpCurrentKF, float minScore, vector<KeyFrame *> vpCandidateKFs) {
        int mnCovisibilityConsistencyTh = 3;

        // Compute reference BoW similarity score
        // This is the lowest score to a connected keyframe in the covisibility graph
        // We will impose loop candidates to have a higher similarity than this
        const vector<KeyFrame *> vpConnectedKeyFrames = mpCurrentKF->GetVectorCovisibleKeyFrames();

        // If there are no loop candidates, just add new keyframe and return false
        if (vpCandidateKFs.empty()) {
            mvConsistentGroupsMap[mpCurrentKF->mpMap->mnId].clear();
            return false;
        }

        // For each loop candidate check consistency with previous loop candidates
        // Each candidate expands a covisibility group (keyframes connected to the loop candidate in the covisibility graph)
        // A group is consistent with a previous group if they share at least a keyframe
        // We must detect a consistent loop in several consecutive keyframes to accept it
        mvpEnoughConsistentCandidatesMap[mpCurrentKF->mpMap->mnId].clear();

        vector<ConsistentGroup> vCurrentConsistentGroups;
        vector<bool> vbConsistentGroup(mvConsistentGroupsMap[mpCurrentKF->mpMap->mnId].size(), false);
        for (size_t i = 0, iend = vpCandidateKFs.size(); i < iend; i++) {
            KeyFrame *pCandidateKF = vpCandidateKFs[i];

            set<KeyFrame *> spCandidateGroup = pCandidateKF->GetConnectedKeyFrames();
            spCandidateGroup.insert(pCandidateKF);

            bool bEnoughConsistent = false;
            bool bConsistentForSomeGroup = false;
            for (size_t iG = 0, iendG = mvConsistentGroupsMap[mpCurrentKF->mpMap->mnId].size(); iG < iendG; iG++) {
                set<KeyFrame *> sPreviousGroup = mvConsistentGroupsMap[mpCurrentKF->mpMap->mnId][iG].first;

                bool bConsistent = false;
                for (set<KeyFrame *>::iterator sit = spCandidateGroup.begin(), send = spCandidateGroup.end();
                     sit != send; sit++) {
                    if (sPreviousGroup.count(*sit)) {
                        bConsistent = true;
                        bConsistentForSomeGroup = true;
                        break;
                    }
                }

                if (bConsistent) {
                    int nPreviousConsistency = mvConsistentGroupsMap[mpCurrentKF->mpMap->mnId][iG].second;
                    int nCurrentConsistency = nPreviousConsistency + 1;
                    if (!vbConsistentGroup[iG]) {
                        ConsistentGroup cg = make_pair(spCandidateGroup, nCurrentConsistency);
                        vCurrentConsistentGroups.push_back(cg);
                        vbConsistentGroup[iG] = true; //this avoid to include the same group more than once
                    }
                    if (nCurrentConsistency >= mnCovisibilityConsistencyTh && !bEnoughConsistent) {
                        mvpEnoughConsistentCandidatesMap[mpCurrentKF->mpMap->mnId].push_back(pCandidateKF);
                        bEnoughConsistent = true; //this avoid to insert the same candidate more than once
                    }
                }
            }

            // If the group is not consistent with any previous group insert with consistency counter set to zero
            if (!bConsistentForSomeGroup) {
                ConsistentGroup cg = make_pair(spCandidateGroup, 0);
                vCurrentConsistentGroups.push_back(cg);
            }
        }

        // Update Covisibility Consistent Groups
        mvConsistentGroupsMap[mpCurrentKF->mpMap->mnId] = vCurrentConsistentGroups;

        debug("loop candidates - after / before {} / {}", mvpEnoughConsistentCandidatesMap[mpCurrentKF->mpMap->mnId].size(), vpCandidateKFs.size());

        if (mvpEnoughConsistentCandidatesMap[mpCurrentKF->mpMap->mnId].empty()) { return false; }

        return true;
    }

    void Client::RunGlobalBundleAdjustment(Map *pMap1, Map* pMap2) {
        bool bStopGBA = false;
        unsigned long nLoopKF = 0;

        vector<KeyFrame *> vpKF1 = pMap1->GetAllKeyFrames();
        vector<MapPoint *> vpMP1 = pMap1->GetAllMapPoints();

        vector<KeyFrame *> vpKF2 = pMap2->GetAllKeyFrames();
        vector<MapPoint *> vpMP2 = pMap2->GetAllMapPoints();

        vpKF1.insert( vpKF1.end(), vpKF2.begin(), vpKF2.end() );
        vpMP1.insert( vpMP1.end(), vpMP2.begin(), vpMP2.end() );


        info("Start BundleAdjustment");
        Optimizer::BundleAdjustment(vpKF1, vpMP1, 10, &bStopGBA, nLoopKF, false, true);
    }
}
