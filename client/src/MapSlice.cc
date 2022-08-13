//
// Created by Halcao on 2020/7/2.
//

#include "MapSlice.h"

ORB_SLAM2::MapSlice::MapSlice(const std::vector<KeyFrame *> &KFs, const std::vector<MapPoint *> &MPs,
                              const std::vector<MapElementUpdateBase *> &updates) : KFs(KFs), MPs(MPs),
                                                                                    updates(updates) {}
