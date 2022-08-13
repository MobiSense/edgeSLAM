//
// Created by halcao on 19-7-10.
//

#include "GlobalInstance.h"

namespace ORB_SLAM2 {
    grpc::Status
    GlobalInstance::DetectRelocalizationCandidates(::grpc::ServerContext *context, const ::FrameRPC *request,
                                                   ::KeyFramesRPC *response) {
        // TODO(ch): finish this;
//        keyFrameDB->DetectRelocalizationCandidates();
        return grpc::Status::OK;
    }

    grpc::Status
    GlobalInstance::ClearDatabase(::grpc::ServerContext *context, const ::Empty *request, ::Empty *response) {
        keyFrameDB->clear();
        return grpc::Status::OK;
    }

    grpc::Status
    GlobalInstance::DatabaseEraseKeyFrame(::grpc::ServerContext *context, const ::UInt32 *request, ::Empty *response) {
        // TODO(ch): finish this;
        keyFrameDB->erase(request->value());
        return grpc::Status::OK;
    }

    grpc::Status
    GlobalInstance::MapAddKeyFrame(::grpc::ServerContext *context, const ::KeyFrameRPC *request, ::Empty *response) {
        // TODO(ch): finish this;
//        map->AddKeyFrame();
        return grpc::Status::OK;
    }

    grpc::Status
    GlobalInstance::MapAddMapPoint(::grpc::ServerContext *context, const ::MapPointRPC *request, ::Empty *response) {
        // TODO(ch): finish this;
//        map->AddMapPoint();
        return grpc::Status::OK;
    }

    grpc::Status
    GlobalInstance::MapEraseMapPoint(::grpc::ServerContext *context, const ::UInt32 *request, ::Empty *response) {
        map->EraseMapPoint(request->value());
        return grpc::Status::OK;
    }

    grpc::Status
    GlobalInstance::MapEraseKeyFrame(::grpc::ServerContext *context, const ::UInt32 *request, ::Empty *response) {
        map->EraseKeyFrame(request->value());
        return grpc::Status::OK;
    }

    grpc::Status GlobalInstance::MapSetReferenceMapPoints(::grpc::ServerContext *context, const ::MapPointsRPC *request,
                                                          ::Empty *response) {
        // TODO(ch): finish this;
//        map->SetReferenceMapPoints();
        return grpc::Status::OK;
    }

    grpc::Status
    GlobalInstance::GetAllKeyFrames(::grpc::ServerContext *context, const ::Empty *request, ::KeyFramesRPC *response) {
        auto keyframes = map->GetAllKeyFrames();
        // TODO(ch): finish this;
        return grpc::Status::OK;
    }

    grpc::Status
    GlobalInstance::GetAllMapPoints(::grpc::ServerContext *context, const ::Empty *request, ::MapPointsRPC *response) {
       auto points = map->GetAllMapPoints();
        // TODO(ch): finish this
        return grpc::Status::OK;
    }

    grpc::Status GlobalInstance::GetReferenceMapPoints(::grpc::ServerContext *context, const ::Empty *request,
                                                       ::MapPointsRPC *response) {
        auto points = map->GetReferenceMapPoints();
//        for (auto itr = points.begin(); itr != points.end(); itr++) {
//
//        }
        // TODO(ch): set mappoints

        return grpc::Status::OK;
    }

    grpc::Status
    GlobalInstance::MapPointsInMap(::grpc::ServerContext *context, const ::Empty *request, ::UInt32 *response) {
        response->set_value(map->MapPointsInMap());
        return grpc::Status::OK;
    }

    grpc::Status
    GlobalInstance::KeyFramesInMap(::grpc::ServerContext *context, const ::Empty *request, ::UInt32 *response) {
        response->set_value(map->KeyFramesInMap());
        return grpc::Status::OK;
    }

    grpc::Status GlobalInstance::MapClear(::grpc::ServerContext *context, const ::Empty *request, ::Empty *response) {
        map->clear();
        return grpc::Status::OK;
    }

    grpc::Status GlobalInstance::TryLockMap(::grpc::ServerContext *context, const ::Empty *request, ::Empty *response) {
        if (stub_ == nullptr) {
            stub_ = TrackingRPC::NewStub(grpc::CreateChannel(Config::GetAddress(false, TrackingPort), grpc::InsecureChannelCredentials()));
        }

        unique_lock<mutex> lock(map->mMutexMapUpdate);
        grpc::ClientContext ctx;
        Empty req, res;
        grpc::Status status = stub_->ContinueTrack(&ctx, req, &res);
        return grpc::Status::OK;
    }
}