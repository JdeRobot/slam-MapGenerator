//
// Created by ernest on 18-6-22.
//

#ifndef SLAM_MAPGEN_LOOPCONNECTION_H
#define SLAM_MAPGEN_LOOPCONNECTION_H

#include "KeyFrame.h"
#include <opencv2/core/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/opencv.hpp>
#include <exception>
#include "logging_util.h"

namespace MapGen {

    class LoopConnection {
    public:
        LoopConnection(KeyFrame * frame_a, KeyFrame * frame_b, const cv::Mat& K, double scaling_factor);

        // get the relative pose between two frames
        cv::Mat get_essential();
        cv::Mat get_translation();
        cv::Mat get_rotation();

    private:
        // the rotation and translation matrix between two matrix
        cv::Mat r_;
        cv::Mat t_;
    };

}


#endif //SLAM_MAPGEN_LOOPCONNECTION_H
