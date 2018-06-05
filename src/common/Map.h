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

#ifndef SLAM_MAPGEN_MAP_H
#define SLAM_MAPGEN_MAP_H

#include <set>
#include <mutex>
#include "MapPoint.h"
#include "KeyFrame.h"

namespace MapGen {

class MapPoint;
class KeyFrame;

class Map {
 public:
    Map();

    std::vector<KeyFrame*> GetAllKeyFrames();
    std::vector<MapPoint*> GetAllMapPoints();

    // Get KeyFrame by id
    KeyFrame* GetKeyFrame(int id);

    // Add KeyFrames and MapPoints
    void AddKeyFrame(KeyFrame* kf);
    void AddMapPoint(MapPoint* mp);

    // Remove KeyFrames and MapPoints
    void EraseMapPoint(MapPoint* mp);
    void EraseKeyFrame(KeyFrame* kf);

    // Update connected KeyFrames
    void UpdateConnections();

    // Clear saved data
    void clear();

 private:
    std::set<MapPoint*> mapPoints_;
    std::set<KeyFrame*> keyFrames_;

    std::mutex mmutexMap_;
};

}

#endif  // SLAM_MAPGEN_MAP_H
