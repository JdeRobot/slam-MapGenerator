//
// Created by ernest on 18-7-11.
//

#ifndef SLAM_MAPGEN_POSE_GRAPH_3D_H
#define SLAM_MAPGEN_POSE_GRAPH_3D_H

#include <iostream>
#include <fstream>
#include <string>

#include <ceres/ceres.h>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include "pose_graph_3d_error_term.h"
#include "types.h"

namespace MapGen{

    void BuildOptimizationProblem(const VectorOfConstraints &constraints,
                                  MapOfPoses *poses, ceres::Problem *problem);

    bool SolveOptimizationProblem(ceres::Problem *problem);

    bool OutputPoses(const std::string &filename, const MapOfPoses &poses);

}

#endif //SLAM_MAPGEN_POSE_GRAPH_3D_H
