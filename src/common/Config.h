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

 /*
  * Developement Log:
  *     TODO: rewrite the ReadParameter so that it is a static function, and separate it from the UI stuff
  */

#ifndef SLAM_MAPGEN_CONFIG_H_
#define SLAM_MAPGEN_CONFIG_H_

#include <iostream>
#include <string>
#include <vector>
#include <opencv2/core/core.hpp>
#include <boost/log/trivial.hpp>
#include "Map.h"

namespace MapGen {

struct CameraParameters {
    double fx;
    double fy;
    double cx;
    double cy;
    double k1;
    double k2;
    double p1;
    double p2;
    double k3;
};

class Config {
 public:
    // Singleton
    static Config& GetInstance() {
        static Config instance;
        return instance;
    }

    // Read parameters from file
    static bool ReadParameters(std::string filename, Map &map);

    // Get UI parameters
    static double KeyFrameSize() { return GetInstance().kKeyFrameSize_; }
    static double KeyFrameLineWidth() { return GetInstance().kKeyFrameLineWidth_; }
    static double GraphLineWidth() { return GetInstance().kGraphLineWidth_; }
    static double PointSize() { return GetInstance().kPointSize_; }
    static double CameraSize() { return GetInstance().kCameraSize_; }
    static double CameraLineWidth() { return GetInstance().kCameraLineWidth_; }
    static double ViewpointX() { return GetInstance().kViewpointX_; }
    static double ViewpointY() { return GetInstance().kViewpointY_; }
    static double ViewpointZ() { return GetInstance().kViewpointZ_; }
    static double ViewpointF() { return GetInstance().kViewpointF_; }

 private:
    Config();

    // UI
    double kKeyFrameSize_;
    double kKeyFrameLineWidth_;
    double kGraphLineWidth_;
    double kPointSize_;
    double kCameraSize_;
    double kCameraLineWidth_;
    double kViewpointX_;
    double kViewpointY_;
    double kViewpointZ_;
    double kViewpointF_;
};

}  // namespace SLAM_MAPGEN


#endif  // SLAM_MAPGEN_CONFIG_H_
