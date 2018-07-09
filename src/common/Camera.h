//
// Created by ernest on 18-6-27.
//

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

        cv::Mat get_intrinsic_matrix();


    private:
        CameraParameters params_;
    };

}


#endif //SLAM_MAPGEN_CAMERA_H
