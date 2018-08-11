/**
 *
 *  Copyright (C) 2018 Jianxiong Cai <caijx AT shanghaitech.edu.cn>
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

#ifndef SLAM_MAPGEN_LOOPCONNECTION_H
#define SLAM_MAPGEN_LOOPCONNECTION_H

#include <opencv2/core/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/opencv.hpp>
#include <exception>
#include "KeyFrame.h"
#include "logging_util.h"
#include "Connection.h"
#include "Camera.h"

namespace MapGen {

    class LoopConnection : Connection {
    public:
        LoopConnection(KeyFrame * frame_a, KeyFrame * frame_b, const Camera& cam) ;

        // get the relative pose between two frames
        cv::Mat get_translation();
        cv::Mat get_rotation();

    private:
        // the rotation and translation matrix between two matrix
        cv::Mat r_;
        cv::Mat t_;
    };

}


#endif //SLAM_MAPGEN_LOOPCONNECTION_H
