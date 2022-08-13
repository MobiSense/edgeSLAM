//
// Created by Halcao on 2020/4/20.
//

#include "MapUpdater.h"
#include "Map.h"
#include "BoostArchiver.h"
#include "CLogger.h"

namespace ORB_SLAM2 {
    map<string, KeyFrameUpdateHandler> MapUpdater::kfHandlerMap = initKeyFrameHandler();
    map<string, MapPointUpdateHandler> MapUpdater::mpHandlerMap = initMapPointHandler();
    map<string, MapEventUpdateHandler> MapUpdater::mapHandlerMap = initMapHandler();

    map<string, KeyFrameUpdateHandler> MapUpdater::initKeyFrameHandler() {
        map<string, KeyFrameUpdateHandler> map;

        map["SetPose"] = [](KeyFrame *kf, MapElementUpdateBase *update) {
            auto arg = dynamic_cast<KeyFrameUpdate<cv::Mat> *>(update)->arg;
            trace("keyframe: {} SetPose", kf->mnId);
            kf->SetPose(arg, false);
        };

        map["AddConnection"] = [](KeyFrame *kf, MapElementUpdateBase *update) {
            auto arg = dynamic_cast<KeyFrameUpdate<std::pair<unsigned long, int> > *>(update)->arg;
            auto target = kf->mpMap->GetKeyFrame(arg.first);
            if (!target) {
                // TODO(halcao): handle it
                warn("AddConnection kf {} null", arg.first);
            }
            kf->AddConnection(target, arg.second, false);
        };

        map["AddMapPoint"] = [](KeyFrame *kf, MapElementUpdateBase *update) {
            auto arg = dynamic_cast<KeyFrameUpdate<std::pair<unsigned long, size_t> > *>(update)->arg;
            auto mp = kf->mpMap->GetMapPoint(arg.first);
            if (!mp) {
                // TODO(halcao): how to deal with it
                warn("AddMapPoint mappoint {} null", arg.first);
                return;
            }
            kf->AddMapPoint(mp, arg.second, false);
        };

        map["EraseMapPointMatch"] = [](KeyFrame *kf, MapElementUpdateBase *update) {
            auto arg = dynamic_cast<KeyFrameUpdate<size_t> *>(update)->arg;
            kf->EraseMapPointMatch(arg, false);
        };

        map["ReplaceMapPointMatch"] = [](KeyFrame *kf, MapElementUpdateBase *update) {
            auto arg = dynamic_cast<KeyFrameUpdate<std::pair<size_t, unsigned long> > *>(update)->arg;
            kf->ReplaceMapPointMatch(arg.first, kf->mpMap->GetMapPoint(arg.second), false);
        };

        map["UpdateConnections"] = [](KeyFrame *kf, MapElementUpdateBase *update) {
//            auto arg = dynamic_cast<KeyFrameUpdate<int> *>(update)->arg;
            kf->UpdateConnections(false);
        };

        map["AddLoopEdge"] = [](KeyFrame *kf, MapElementUpdateBase *update) {
            auto arg = dynamic_cast<KeyFrameUpdate<unsigned long> *>(update)->arg;
            kf->AddLoopEdge(kf->mpMap->GetKeyFrame(arg), false);
        };

        map["SetBadFlag"] = [](KeyFrame *kf, MapElementUpdateBase *update) {
            kf->SetBadFlag(false);
        };

        return map;
    }

    map<string, MapPointUpdateHandler> MapUpdater::initMapPointHandler() {
        map<string, MapPointUpdateHandler> map;

        map["SetWorldPos"] = [](MapPoint *mp, MapElementUpdateBase *update) {
            auto arg = dynamic_cast<MapPointUpdate<cv::Mat> *>(update)->arg;
//            cout << "mappoint: " << mp->mnId << " SetWorldPos\n";
            mp->SetWorldPos(arg, false);
        };

        map["AddObservation"] = [](MapPoint *mp, MapElementUpdateBase *update) {
            auto arg = dynamic_cast<MapPointUpdate<std::pair<unsigned long, size_t> > *>(update)->arg;
            auto kf = mp->mpMap->GetKeyFrame(arg.first);
            if (!kf) {
                // TODO(halcao): handle it
                warn("AddObservation kf {} null", arg.first);
                return;
            }
            mp->AddObservation(kf, arg.second, false);
        };

        map["EraseObservation"] = [](MapPoint *mp, MapElementUpdateBase *update) {
            auto arg = dynamic_cast<MapPointUpdate<unsigned long> *>(update)->arg;
            mp->EraseObservation(mp->mpMap->GetKeyFrame(arg), false);
        };

        map["SetBadFlag"] = [](MapPoint *mp, MapElementUpdateBase *update) {
//            auto arg = dynamic_cast<MapPointUpdate<unsigned long> *>(update)->arg;
            mp->SetBadFlag(false);
        };

        map["Replace"] = [](MapPoint *mp, MapElementUpdateBase *update) {
            auto arg = dynamic_cast<MapPointUpdate<unsigned long> *>(update)->arg;
            auto target = mp->mpMap->GetMapPoint(arg);
            if (target) {
                mp->Replace(target, false);
            } else {
                warn("mp {} replace {} null", mp->mnId, arg);
            }
        };

        map["ComputeDistinctiveDescriptors"] = [](MapPoint *mp, MapElementUpdateBase *update) {
            mp->ComputeDistinctiveDescriptors(false);
        };

        map["UpdateNormalAndDepth"] = [](MapPoint *mp, MapElementUpdateBase *update) {
            mp->UpdateNormalAndDepth(false);
        };

        map["IncreaseVisible"] = [](MapPoint *mp, MapElementUpdateBase *update) {
            auto arg = dynamic_cast<MapPointUpdate<int> *>(update)->arg;
            mp->IncreaseVisible(arg, false);
        };

        map["IncreaseFound"] = [](MapPoint *mp, MapElementUpdateBase *update) {
            auto arg = dynamic_cast<MapPointUpdate<int> *>(update)->arg;
            mp->IncreaseFound(arg, false);
        };

        return map;
    }

    map<string, MapEventUpdateHandler> MapUpdater::initMapHandler() {
        map<string, MapEventUpdateHandler> map;

        map["SetReferenceMapPoints"] = [](Map *pMap, MapElementUpdateBase *update) {
            auto arg = dynamic_cast<MapEventUpdate<vector<unsigned long> > *>(update)->arg;
            vector<MapPoint *> vpMPs;
            for (auto &id: arg) {
                auto mp = pMap->GetMapPoint(id);
                if (mp) {
                    vpMPs.push_back(mp);
                }
            }
            pMap->SetReferenceMapPoints(vpMPs, false);
        };

        map["clear"] = [](Map *pMap, MapElementUpdateBase *update) {
//            auto arg = dynamic_cast<MapEventUpdate<int> *>(update)->arg;
            info("map {} cleared", pMap->mnId);
            pMap->clear(false);
        };

        map["InformNewBigChange"] = [](Map *pMap, MapElementUpdateBase *update) {
//            auto arg = dynamic_cast<MapEventUpdate<int> *>(update)->arg;
            pMap->InformNewBigChange(false);
        };

        // TODO(halcao): validate this
        map["AddLoopClosing"] = [](Map *pMap, MapElementUpdateBase *update) {
            auto arg = dynamic_cast<MapEventUpdate<unsigned long> *>(update)->arg;

            auto kf = pMap->GetKeyFrame(arg);
            if (!kf) {
                warn("AddLoopClosing kf {} null", arg);
                return;
            }
            pMap->mpLoopCloser->InsertKeyFrame(kf);
            debug("kf {} added to map {} loop closing", arg, pMap->mnId);
//            pMap->mvLoopClosingQueue.push_back(arg);
        };

        return map;
    }

    vector<unsigned int> MapUpdater::Apply(Map *pMap, vector<MapElementUpdateBase *> updates) {
//        auto originUpdates = pMap->DequeueUpdates();
        vector<unsigned int> successIds;

        for (auto &update: updates) {
            if (!update || update->mnId == (unsigned long) (-1)) {
                warn("empty update with mnId {} of func: {}", update->mnId, update->funcName);
                continue;
            }

            switch (update->getType()) {
                case KeyFrameType: {
                    auto kf = pMap->GetKeyFrame(update->mnId);
                    if (kf && kfHandlerMap.count(update->funcName)) {
                        auto func = kfHandlerMap[update->funcName];
                        func(kf, update);
                        successIds.push_back(update->id);
                    }
                    break;
                }
                case MapPointType: {
                    auto mp = pMap->GetMapPoint(update->mnId);
                    if (mp && mpHandlerMap.count(update->funcName)) {
                        auto func = mpHandlerMap[update->funcName];
                        func(mp, update);
                        successIds.push_back(update->id);
                    }
                    break;
                }
                case MapEventType: {
//                    auto targetMap = Map::GetMap(update->mnId);
                    auto func = mapHandlerMap[update->funcName];
                    if (func) {
                        func(pMap, update);
                        successIds.push_back(update->id);
                    }
                    break;
                }
            }
        }

        // TODO(halcao): refactor it
//        pMap->DequeueUpdates();
//        for (auto &update: originUpdates) {
//            pMap->AddUpdate(update);
//        }
        return successIds;
    }


    template<class Archive>
    void MapUpdater::RegisterType(Archive &ar) {
        // The template derived types should be registered first by the requirement of Boost Serialization
        ar.template register_type<KeyFrameUpdate<cv::Mat> >();
        ar.template register_type<KeyFrameUpdate<std::pair<unsigned long, int> > >();
        ar.template register_type<KeyFrameUpdate<std::pair<unsigned long, size_t> >>();
        ar.template register_type<KeyFrameUpdate<size_t> >();
        ar.template register_type<KeyFrameUpdate<std::pair<size_t, unsigned long> > >();
        ar.template register_type<KeyFrameUpdate<unsigned long> >();
        ar.template register_type<KeyFrameUpdate<int> >();

        ar.template register_type<MapPointUpdate<cv::Mat> >();
        ar.template register_type<MapPointUpdate<std::pair<unsigned long, size_t> >>();
        ar.template register_type<MapPointUpdate<unsigned long> >();
        ar.template register_type<MapPointUpdate<int> >();

        ar.template register_type<MapEventUpdate<unsigned long> >();
        ar.template register_type<MapEventUpdate<vector<unsigned long>>>();
        ar.template register_type<MapEventUpdate<int> >();
    }

    // The template function should declare their available types
    template void MapUpdater::RegisterType(boost::archive::binary_iarchive &);
    template void MapUpdater::RegisterType(boost::archive::text_iarchive &);
    template void MapUpdater::RegisterType(boost::archive::binary_oarchive &);
    template void MapUpdater::RegisterType(boost::archive::text_oarchive &);
}