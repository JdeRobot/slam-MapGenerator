/**
 *
 *  Copyright (C) 2018 Jianxiong Cai <caijx AT shanghaitech.edu.cn>
 *
 *  The following code is a derivative work of the code from the slam-viewer project,
 *  which is licensed under the GNU Public License, version 3. This code therefore
 *  is also licensed under the terms of the GNU Public License, version 3.
 *  For more information see <https://github.com/JdeRobot/slam-viewer>.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 *  GNU Library General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 */


#include "MapPoint.h"

using std::mutex;
using std::unique_lock;

namespace MapGen {

MapPoint::MapPoint(int id, const std::vector<double> &position): id_(id) {
    assert(position.size() == 3);
    position_(0) = position[0];
    position_(1) = position[1];
    position_(2) = position[2];
}

Eigen::Vector3d MapPoint::GetWorldPos() {
    unique_lock<mutex> lock(mutexPos_);
    return position_;
}

void MapPoint::AddObservation(KeyFrame* kf) {
    unique_lock<mutex> lock(mutexFeatures_);
    observations_.insert(kf);
}

void MapPoint::EraseObservation(KeyFrame* kf) {
    unique_lock<mutex> lock(mutexFeatures_);

    if (observations_.count(kf))
        observations_.erase(kf);
}


std::set<KeyFrame*> MapPoint::GetObservations() {
    unique_lock<mutex> lock(mutexFeatures_);
    return observations_;
}

}  // namespace SLAM_VIEWER
