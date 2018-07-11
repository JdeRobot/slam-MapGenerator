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
