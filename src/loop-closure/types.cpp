#include "types.h"

namespace MapGen{

    Pose3d::Pose3d(const Eigen::Matrix4d& pose) {
        p = pose.block<3,1>(0,3).col(0);

        Eigen::Matrix3d rotation = pose.block<3,3>(0,0);
        q = Eigen::Quaterniond(rotation);
        q.normalize();
    }

    std::string Pose3d::name() {
        return "VERTEX_SE3:QUAT";
    }

    Constraint3d::Constraint3d(int id_1, int id_2, const Eigen::Matrix4d &t): id_begin(id_1), id_end(id_2), t_be(t) {
        information = Eigen::Matrix<double,6,6>::Identity();
    }

    std::string Constraint3d::name() {
        return "EDGE_SE3:QUAT";
    }
}