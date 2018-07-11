//
// Created by ernest on 18-6-22.
//

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
