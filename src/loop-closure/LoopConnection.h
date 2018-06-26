//
// Created by ernest on 18-6-22.
//

#ifndef SLAM_MAPGEN_LOOPCONNECTION_H
#define SLAM_MAPGEN_LOOPCONNECTION_H

#include "KeyFrame.h"
#include <opencv2/core/core.hpp>
#include <opencv2/features2d.hpp>

namespace MapGen {

    class LoopConnection {
    public:
        LoopConnection(const std::pair<KeyFrame *, KeyFrame *>& loop_closing_pair,
                       const std::map<int, cv::Mat>& feature_map, cv::BFMatcher& matcher);

        
    private:
        // the essential matrix between two frames
        cv::Mat sle;
    };

}


#endif //SLAM_MAPGEN_LOOPCONNECTION_H
