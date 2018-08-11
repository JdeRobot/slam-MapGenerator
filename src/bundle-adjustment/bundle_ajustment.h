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

#ifndef SLAM_MAPGEN_BUNDLEAJUSTMENT_H
#define SLAM_MAPGEN_BUNDLEAJUSTMENT_H


#include <cmath>
#include <cstdio>
#include <iostream>
#include <pcl/common/eigen.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

#include "ceres/ceres.h"
#include "ceres/rotation.h"
#include "Map.h"
#include "Camera.h"


namespace MapGen {

// Read a Bundle Adjustment in the Large dataset.
    class BALProblem {
    public:
        ~BALProblem();

        int num_observations() const;

        const double *observations() const;

        double *mutable_cameras();

        double *mutable_points();

        double *mutable_camera_for_observation(int i);

        double *mutable_point_for_observation(int i);

        bool LoadFile(const char *filename);

        bool LoadFromMap(Map& map, const Camera& cam);

        void SaveToMap(Map& map, const Camera& cam);
    private:
        template<typename T>
        void FscanfOrDie(FILE *fptr, const char *format, T *value);

        int num_cameras_;
        int num_points_;
        int num_observations_;
        int num_parameters_;

        int *point_index_;
        int *camera_index_;
        double *observations_;
        double *parameters_;

        double camera_focus_;
    };

// Templated pinhole camera model for used with Ceres.  The camera is
// parameterized using 9 parameters: 3 for rotation, 3 for translation, 1 for
// focal length and 2 for radial distortion. The principal point is not modeled
// (i.e. it is assumed be located at the image center).
    struct SnavelyReprojectionError {
        SnavelyReprojectionError(double observed_x, double observed_y, const CameraParameters cam);

        template<typename T>
        bool operator()(const T *const camera,
                        const T *const point,
                        T *residuals) const;

        // Factory to hide the construction of the CostFunction object from
        // the client code.
        static ceres::CostFunction *Create(const double observed_x,
                                           const double observed_y,
                                           const CameraParameters cam
        );

        double observed_x;
        double observed_y;

        double camera_fx;
        double camera_fy;
        double camera_cx;
        double camera_cy;
    };

}


#endif //SLAM_MAPGEN_BUNDLEAJUSTMENT_H
