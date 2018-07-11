//
// Created by ernest on 18-7-11.
//

#include "Connection.h"


namespace MapGen{

    Connection::Connection() : frame_a_(NULL), frame_b_(NULL), T_(Eigen::Matrix4d::Identity()) {}

    Connection::Connection(KeyFrame *frame_a, KeyFrame *frame_b, const Camera &cam) :
            frame_a_(frame_a), frame_b_(frame_b) {
//        std::cout << "Pose frame_a: " << std::endl;
//        std::cout << frame_a->GetPose() << std::endl;
        T_ = frame_a->GetPose().inverse() * frame_b->GetPose();
    }

    Eigen::Matrix4d Connection::get_sim3() {
        return T_;
    }
}