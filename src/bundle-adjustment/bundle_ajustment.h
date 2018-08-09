// Ceres Solver - A fast non-linear least squares minimizer
// Copyright 2015 Google Inc. All rights reserved.
// http://ceres-solver.org/
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
// * Neither the name of Google Inc. nor the names of its contributors may be
//   used to endorse or promote products derived from this software without
//   specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// Author: keir@google.com (Keir Mierle)
//
// A minimal, self-contained bundle adjuster using Ceres, that reads
// files from University of Washington' Bundle Adjustment in the Large dataset:
// http://grail.cs.washington.edu/projects/bal
//
// This does not use the best configuration for solving; see the more involved
// bundle_adjuster.cc file for details.

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
