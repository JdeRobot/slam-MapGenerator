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

#include "Map.h"

using std::mutex;
using std::unique_lock;
using std::vector;
using std::set;

namespace MapGen {

Map::Map() {
}

KeyFrame* Map::GetKeyFrame(int id) {
    unique_lock<mutex> lock(mmutexMap_);
    for (auto it=keyFrames_.begin(); it!=keyFrames_.end(); it++) {
        if ((*it)->GetId() == id)
            return *it;
    }

    return nullptr;
}

void Map::AddKeyFrame(KeyFrame *kf) {
    unique_lock<mutex> lock(mmutexMap_);
    keyFrames_.insert(kf);
}

void Map::AddMapPoint(MapPoint *mp) {
    unique_lock<mutex> lock(mmutexMap_);
    mapPoints_.insert(mp);
}

void Map::EraseMapPoint(MapPoint *mp) {
    unique_lock<mutex> lock(mmutexMap_);
    mapPoints_.erase(mp);
}

void Map::EraseKeyFrame(KeyFrame *kf) {
    unique_lock<mutex> lock(mmutexMap_);
    keyFrames_.erase(kf);
}

vector<KeyFrame*> Map::GetAllKeyFrames() {
    unique_lock<mutex> lock(mmutexMap_);
    return vector<KeyFrame*>(keyFrames_.begin(), keyFrames_.end());
}

vector<MapPoint*> Map::GetAllMapPoints() {
    unique_lock<mutex> lock(mmutexMap_);
    return vector<MapPoint*>(mapPoints_.begin(), mapPoints_.end());
}

void Map::clear() {
    unique_lock<mutex> lock(mmutexMap_);
    for (auto it = mapPoints_.begin(), it_end = mapPoints_.end(); it != it_end; it++)
        delete *it;

    for (auto it = keyFrames_.begin(), it_end = keyFrames_.end(); it != it_end; it++)
        delete *it;

    mapPoints_.clear();
    keyFrames_.clear();
}

void Map::UpdateConnections() {
    unique_lock<mutex> lock(mmutexMap_);

    for (auto it = keyFrames_.begin(), it_end = keyFrames_.end(); it != it_end; it++)
        (*it)->UpdateConnections();
}

}  // namespace SLAM_VIEWER
