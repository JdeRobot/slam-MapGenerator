/**
 *
 *  Copyright (C) 2018 Jianxiong Cai <caijx AT shanghaitech.edu.cn>
 *
 *  The following code is a derivative work of the code from the Ceres
 *  Solver project. Thus original license for ceres solver is also
 *  included following in the end. For more information, see
 *  <http://ceres-solver.org/>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 *  GNU Library General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 *
 *  Ceres Solver - A fast non-linear least squares minimizer
 *  Copyright 2016 Google Inc. All rights reserved.
 *  http://ceres-solver.org/
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *  * Neither the name of Google Inc. nor the names of its contributors may be
 *    used to endorse or promote products derived from this software without
 *    specific prior written permission.
 */


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
