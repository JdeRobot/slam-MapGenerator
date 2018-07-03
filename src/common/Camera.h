//
// Created by ernest on 18-6-27.
//

#ifndef SLAM_MAPGEN_CAMERA_H
#define SLAM_MAPGEN_CAMERA_H

#include <opencv2/core.hpp>
#include "Config.h"

namespace MapGen {

    class Camera {
    public:
        Camera(const CameraParameters& params);

        cv::Point2d get_priciple_points();


    private:
        cv::Mat intrinsic_mat_;
        cv::Mat distortion_;

        CameraParameters params_;
    };

}


#endif //SLAM_MAPGEN_CAMERA_H
