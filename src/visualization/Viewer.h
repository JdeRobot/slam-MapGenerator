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

#ifndef SLAM_VIEWER_VIEWER_H
#define SLAM_VIEWER_VIEWER_H

#include <mutex>
#include <string>
#include <pangolin/pangolin.h>
#include <pangolin/image/image_io.h>
#include <unistd.h>
#include "Config.h"
#include "MapDrawer.h"
#include "logging_util.h"
#include "SOIL.h"

namespace MapGen {

class MapDrawer;
class System;

class Viewer {
 public:
    Viewer(MapDrawer* pMapDrawer);

    // Main thread function. Draw points, keyframes, and connections. We use Pangolin.
    void Run();

    void RequestFinish();

    bool isFinished();

 private:
    MapDrawer* mpMapDrawer;

    bool CheckFinish();
    void SetFinish();
    bool mbFinishRequested;
    bool mbFinished;
    std::mutex mMutexFinish;
};

}  // namespace SLAM_VIEWER

#endif  // SLAM_VIEWER_VIEWER_H

