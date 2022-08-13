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

#ifndef LOCALMAPPING_H
#define LOCALMAPPING_H

#include "KeyFrame.h"
#include "Map.h"
#include "LoopClosing.h"
#include "Tracking.h"
#include "KeyFrameDatabase.h"

#include <mutex>

#include "LocalMapping.grpc.pb.h"


namespace ORB_SLAM2
{

class Tracking;
class LoopClosing;
class Map;

class LocalMapping final : LocalMappingRPC::Service
{
public:
    LocalMapping(Map* pMap, const float bMonocular);

    void SetLoopCloser(LoopClosing* pLoopCloser);

    void SetTracker(Tracking* pTracker);

    // Main function
    void Run();

    void InsertKeyFrame(KeyFrame* pKF);

    // Thread Synch
    void RequestStop();
    void RequestReset();
    bool Stop();
    void Release();
    bool isStopped();
    bool stopRequested();
    bool AcceptKeyFrames();
    void SetAcceptKeyFrames(bool flag);
    bool SetNotStop(bool flag);

    void InterruptBA();

    void RequestFinish();
    bool isFinished();

    int KeyframesInQueue(){
        unique_lock<std::mutex> lock(mMutexNewKFs);
        return mlNewKeyFrames.size();
    }


    grpc::Status Run(::grpc::ServerContext *context, const ::Empty *request, ::Empty *response) override;

    grpc::Status
    InsertKeyFrame(::grpc::ServerContext *context, const ::KeyFrameRPC *request, ::Empty *response) override;

    grpc::Status RequestStop(::grpc::ServerContext *context, const ::Empty *request, ::Empty *response) override;

    grpc::Status RequestReset(::grpc::ServerContext *context, const ::Empty *request, ::Empty *response) override;

    grpc::Status Release(::grpc::ServerContext *context, const ::Empty *request, ::Empty *response) override;

    grpc::Status isStopped(::grpc::ServerContext *context, const ::Empty *request, ::Bool *response) override;

    grpc::Status stopRequested(::grpc::ServerContext *context, const ::Empty *request, ::Bool *response) override;

    grpc::Status AcceptKeyFrames(::grpc::ServerContext *context, const ::Empty *request, ::Bool *response) override;

    grpc::Status SetNotStop(::grpc::ServerContext *context, const ::Bool *request, ::Bool *response) override;

    grpc::Status InterruptBA(::grpc::ServerContext *context, const ::Empty *request, ::Empty *response) override;

    grpc::Status RequestFinish(::grpc::ServerContext *context, const ::Empty *request, ::Empty *response) override;

    grpc::Status isFinished(::grpc::ServerContext *context, const ::Empty *request, ::Bool *response) override;

    grpc::Status KeyframesInQueue(::grpc::ServerContext *context, const ::Empty *request, ::UInt32 *response) override;

protected:

    bool CheckNewKeyFrames();
    void ProcessNewKeyFrame();
    void CreateNewMapPoints();

    void MapPointCulling();
    void SearchInNeighbors();

    void KeyFrameCulling();

    cv::Mat ComputeF12(KeyFrame* &pKF1, KeyFrame* &pKF2);

    cv::Mat SkewSymmetricMatrix(const cv::Mat &v);

    bool mbMonocular;

    void ResetIfRequested();
    bool mbResetRequested;
    std::mutex mMutexReset;

    bool CheckFinish();
    void SetFinish();
    bool mbFinishRequested;
    bool mbFinished;
    std::mutex mMutexFinish;

    Map* mpMap;

    LoopClosing* mpLoopCloser;
    Tracking* mpTracker;

    std::list<KeyFrame*> mlNewKeyFrames;

    KeyFrame* mpCurrentKeyFrame;

    std::list<MapPoint*> mlpRecentAddedMapPoints;

    std::mutex mMutexNewKFs;

    bool mbAbortBA;

    bool mbStopped;
    bool mbStopRequested;
    bool mbNotStop;
    std::mutex mMutexStop;

    bool mbAcceptKeyFrames;
    std::mutex mMutexAccept;
};

} //namespace ORB_SLAM

#endif // LOCALMAPPING_H
