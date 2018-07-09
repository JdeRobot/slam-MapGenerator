//
// Created by ernest on 18-6-22.
//

#include "LoopConnection.h"



namespace MapGen{

    // calculate the fundamental matrix, then get the essential matrix
    LoopConnection::LoopConnection(const std::pair<KeyFrame *, KeyFrame *> &loop_closing_pair,
                                   cv::BFMatcher &matcher) {
        throw std::runtime_error("No implementation yet!");
    }

    cv::Mat LoopConnection::get_sim3() {
        return sle_;
    }
}