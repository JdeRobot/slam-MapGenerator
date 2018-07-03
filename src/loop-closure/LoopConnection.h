//
// Created by ernest on 18-6-22.
//

#ifndef SLAM_MAPGEN_LOOPCONNECTION_H
#define SLAM_MAPGEN_LOOPCONNECTION_H

#include "KeyFrame.h"
#include <opencv2/core/core.hpp>
#include <opencv2/features2d.hpp>
#include <exception>

namespace MapGen {

    class LoopConnection {
    public:
        LoopConnection(const std::pair<KeyFrame *, KeyFrame *>& loop_closing_pair, cv::BFMatcher& matcher);

        // get the relative pose between two frames
        cv::Mat get_sim3();

    private:
        // the essential matrix between two frames
        cv::Mat sle_;
    };

}


#endif //SLAM_MAPGEN_LOOPCONNECTION_H
