//
// Created by halcao on 19-7-10.
//

#ifndef EDGE_SLAM_GLOBALINSTANCE_H
#define EDGE_SLAM_GLOBALINSTANCE_H

#include "GlobalInstance.grpc.pb.h"
#include "KeyFrameDatabase.h"
#include "GRPCConfig.h"
#include "Tracking.grpc.pb.h"
#include <grpcpp/grpcpp.h>

namespace ORB_SLAM2 {

    class GlobalInstance final : public GlobalInstanceRPC::Service {
    public:
        static GlobalInstance &getInstance() {
            static GlobalInstance instance;
            return instance;
        }

        GlobalInstance(GlobalInstance const &) = delete;

        void operator=(GlobalInstance const &) = delete;

        Map *getMap() const {
            return map;
        }

        KeyFrameDatabase *getKeyFrameDb() const {
            return keyFrameDB;
        }

        void setMap(Map *map) {
            GlobalInstance::map = map;
        }

        void setKeyFrameDb(KeyFrameDatabase *keyFrameDb) {
            keyFrameDB = keyFrameDb;
        }

        grpc::Status DetectRelocalizationCandidates(::grpc::ServerContext *context, const ::FrameRPC *request,
                                                    ::KeyFramesRPC *response) override;

        grpc::Status ClearDatabase(::grpc::ServerContext *context, const ::Empty *request, ::Empty *response) override;

        grpc::Status
        DatabaseEraseKeyFrame(::grpc::ServerContext *context, const ::UInt32 *request, ::Empty *response) override;

        grpc::Status
        MapAddKeyFrame(::grpc::ServerContext *context, const ::KeyFrameRPC *request, ::Empty *response) override;

        grpc::Status
        MapAddMapPoint(::grpc::ServerContext *context, const ::MapPointRPC *request, ::Empty *response) override;

        grpc::Status
        MapEraseMapPoint(::grpc::ServerContext *context, const ::UInt32 *request, ::Empty *response) override;

        grpc::Status
        MapEraseKeyFrame(::grpc::ServerContext *context, const ::UInt32 *request, ::Empty *response) override;

        grpc::Status MapSetReferenceMapPoints(::grpc::ServerContext *context, const ::MapPointsRPC *request,
                                              ::Empty *response) override;

        grpc::Status
        GetAllKeyFrames(::grpc::ServerContext *context, const ::Empty *request, ::KeyFramesRPC *response) override;

        grpc::Status
        GetAllMapPoints(::grpc::ServerContext *context, const ::Empty *request, ::MapPointsRPC *response) override;

        grpc::Status GetReferenceMapPoints(::grpc::ServerContext *context, const ::Empty *request,
                                           ::MapPointsRPC *response) override;

        grpc::Status
        MapPointsInMap(::grpc::ServerContext *context, const ::Empty *request, ::UInt32 *response) override;

        grpc::Status
        KeyFramesInMap(::grpc::ServerContext *context, const ::Empty *request, ::UInt32 *response) override;

        grpc::Status MapClear(::grpc::ServerContext *context, const ::Empty *request, ::Empty *response) override;

        grpc::Status TryLockMap(::grpc::ServerContext *context, const ::Empty *request, ::Empty *response) override;
        
    private:
        shared_ptr<TrackingRPC::Stub> stub_;

        GlobalInstance() {
        }

        Map *map;
        KeyFrameDatabase *keyFrameDB;
    };
}
#endif //EDGE_SLAM_GLOBALINSTANCE_H
