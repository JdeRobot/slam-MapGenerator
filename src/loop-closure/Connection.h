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


// Connection between sequential frames

#ifndef SLAM_MAPGEN_CONNECTION_H
#define SLAM_MAPGEN_CONNECTION_H

#include <opencv2/core/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/opencv.hpp>
#include <exception>
#include "KeyFrame.h"
#include "logging_util.h"
#include "Camera.h"

namespace MapGen {

    class Connection {
    public:
        Connection();

        Connection(KeyFrame * frame_a, KeyFrame * frame_b, const Camera &cam);

        Eigen::Matrix4d get_sim3();

    protected:
        // keyframes
        KeyFrame * frame_a_;
        KeyFrame * frame_b_;

        // observations (sim3 transform between 2 frames)
        Eigen::Matrix4d T_;
    };

}


#endif //SLAM_MAPGEN_CONNECTION_H
