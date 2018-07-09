//
// Created by ernest on 18-6-27.
//

#ifndef SLAM_MAPGEN_CAMERA_H
#define SLAM_MAPGEN_CAMERA_H

#include <opencv2/core.hpp>
#include "Config.h"


// The intrinsic matrix K is in the following gorm
// fx  0   cx 
// 0   fy  cy 
// 0   0   1 

namespace MapGen {

    class Camera {
    public:
        Camera(const CameraParameters& params);

        cv::Point2d get_priciple_points();
        cv::Mat get_intrinsic_matrix();


    private:
        CameraParameters params_;
    };

}


#endif //SLAM_MAPGEN_CAMERA_H
