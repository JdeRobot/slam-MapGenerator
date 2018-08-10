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

#ifndef SLAM_MAPGEN_CAMERA_H
#define SLAM_MAPGEN_CAMERA_H

#include <opencv2/core.hpp>
#include "logging_util.h"


// The intrinsic matrix K is in the following gorm
// fx  0   cx 
// 0   fy  cy 
// 0   0   1 

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


    class Camera {
    public:
        Camera();
        explicit Camera(const CameraParameters& params) ;

        cv::Mat get_intrinsic_matrix() const;

        CameraParameters get_camera_params() const;


    private:
        CameraParameters params_;
    };

}


#endif //SLAM_MAPGEN_CAMERA_H
