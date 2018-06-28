//
// Created by ernest on 18-6-22.
//

#include "LoopConnection.h"

namespace MapGen{
    cv::Mat LoopConnection::get_sim3() {
        return sle_;
    }
}