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

#include "bundle_ajustment.h"

namespace MapGen{
    BALProblem::~BALProblem() {
        delete[] point_index_;
        delete[] camera_index_;
        delete[] observations_;
        delete[] parameters_;
    }

    int BALProblem::num_observations() const {
        return num_observations_;
    }

    const double* BALProblem::observations() const {
        return observations_;
    }

    double* BALProblem::mutable_cameras() {
        return parameters_;
    }

    double* BALProblem::mutable_points() {
        return parameters_ + 9 * num_cameras_;
    }

    double* BALProblem::mutable_camera_for_observation(int i) {
        return mutable_cameras() + camera_index_[i] * 9;
    }

    double* BALProblem::mutable_point_for_observation(int i) {
        return mutable_points() + point_index_[i] * 3;
    }

    bool BALProblem::LoadFile(const char *filename) {
        FILE *fptr = fopen(filename, "r");
        if (fptr == NULL) {
            return false;
        };

        FscanfOrDie(fptr, "%d", &num_cameras_);
        FscanfOrDie(fptr, "%d", &num_points_);
        FscanfOrDie(fptr, "%d", &num_observations_);

        point_index_ = new int[num_observations_];
        camera_index_ = new int[num_observations_];
        observations_ = new double[2 * num_observations_];

        num_parameters_ = 9 * num_cameras_ + 3 * num_points_;
        parameters_ = new double[num_parameters_];

        for (int i = 0; i < num_observations_; ++i) {
            FscanfOrDie(fptr, "%d", camera_index_ + i);
            FscanfOrDie(fptr, "%d", point_index_ + i);
            for (int j = 0; j < 2; ++j) {
                FscanfOrDie(fptr, "%lf", observations_ + 2 * i + j);
            }
        }

        for (int i = 0; i < num_parameters_; ++i) {
            FscanfOrDie(fptr, "%lf", parameters_ + i);
        }
        return true;
    }

    template<typename T>
    void BALProblem::FscanfOrDie(FILE *fptr, const char *format, T *value) {
        int num_scanned = fscanf(fptr, format, value);
        if (num_scanned != 1) {
            LOG(FATAL) << "Invalid UW data file.";
        }
    }

    bool BALProblem::LoadFromMap(MapGen::Map &map, const Camera& cam) {

        camera_focus_ = cam.get_camera_params().fx;

        auto keyframes = map.GetAllKeyFrames();
        auto map_points = map.GetAllMapPoints();

        num_cameras_ = keyframes.size();
        num_points_ = map_points.size();
        num_observations_ = 0;
        for (auto frame : keyframes){
            num_observations_ += frame->GetObservations().size();
        }

        // to describe an observation
        point_index_ = new int[num_observations_];
        camera_index_ = new int[num_observations_];
        observations_ = new double[2 * num_observations_];

        // the initial camera poses and point poses
        num_parameters_ = 9 * num_cameras_ + 3 * num_points_;
        parameters_ = new double[num_parameters_];

        // fill in the observations
        int ob_idx = 0;
        for (int kf_idx = 0; kf_idx < keyframes.size(); kf_idx ++){
            auto frame = keyframes[kf_idx];
            auto observations_per_frame = frame->GetObservations();
            for (std::pair<MapPoint *, Eigen::Vector2d> ob : observations_per_frame){
                // use the consisting keyframe index instead of the original KF index
                camera_index_[ob_idx] = kf_idx;
                point_index_[ob_idx] = ob.first->GetID();
                observations_[2*ob_idx] = ob.second(0);
                observations_[2*ob_idx + 1] = ob.second(1);
                ob_idx ++;
            }
        }

        // fill in initial camera poses and point poses
        // camera poses
        for (int kf_idx = 0; kf_idx < keyframes.size(); kf_idx ++){
            Eigen::Matrix3d rotation_mat_eigen = keyframes[kf_idx]->GetRotation().transpose();
            Eigen::Vector3d translation_vec = -1 * keyframes[kf_idx]->GetRotation().transpose() *
                    keyframes[kf_idx]->GetTranslation();

//            Eigen::Vector3d rotation_vec = q.toRotationMatrix().eulerAngles(0,1,2);
//            parameters_[kf_idx*9] = rotation_vec[0];
//            parameters_[kf_idx*9 + 1] = rotation_vec[1];
//            parameters_[kf_idx*9 + 2] = rotation_vec[2];
            cv::Mat rotation_mat(3,3,CV_64F);
            cv::Mat rotation_vec(3,1,CV_64F);

//            Eigen::Matrix3d rotation_mat_eigen = keyframes[kf_idx]->GetRotation();

            cv::eigen2cv(rotation_mat_eigen, rotation_mat);
            // convert rotation matrix to Rodrigues vector
            cv::Rodrigues(rotation_mat,rotation_vec);

//            LOG_INFO << std::endl << rotation_mat_eigen << std::endl;
//            LOG_INFO << std::endl << rotation_mat << std::endl;
//            LOG_INFO << std::endl;
//            LOG_INFO << std::endl << rotation_vec << std::endl;
//
//            LOG_INFO << rotation_vec.at<double>(0) << std::endl;
            parameters_[kf_idx*9] = rotation_vec.at<double>(0);
            parameters_[kf_idx*9 + 1] = rotation_vec.at<double>(1);
            parameters_[kf_idx*9 + 2] = rotation_vec.at<double>(2);

            // translation
//            Eigen::Vector3d translation_vec = keyframes[kf_idx]->GetTranslation();
            parameters_[kf_idx*9 + 3] = translation_vec[0];
            parameters_[kf_idx*9 + 4] = translation_vec[1];
            parameters_[kf_idx*9 + 5] = translation_vec[2];

            // camera intrinsic
            auto intrinsic = cam.get_camera_params();
            parameters_[kf_idx*9 + 6] = intrinsic.fx;
            parameters_[kf_idx*9 + 7] = intrinsic.k1;
            parameters_[kf_idx*9 + 8] = intrinsic.k2;
        }

        // fill in point poses
        int starting_idx = keyframes.size() * 9;
        for (auto point : map_points){
            auto point_id = point->GetID();
            auto point_pos = point->GetWorldPos();
            parameters_[starting_idx + point_id * 3] = point_pos[0];
            parameters_[starting_idx + point_id * 3 + 1] = point_pos[1];
            parameters_[starting_idx + point_id * 3 + 2] = point_pos[2];
        }

        return true;
    }


    void BALProblem::SaveToMap(MapGen::Map &map, const MapGen::Camera &cam) {

        std::vector<KeyFrame *> keyframes = map.GetAllKeyFrames();
        std::vector<MapPoint *> map_points = map.GetAllMapPoints();

        for (int kf_idx = 0; kf_idx < keyframes.size(); kf_idx ++){
            Eigen::Matrix4d pose = Eigen::Matrix4d::Zero(4,4);

            // get rotation matrix
//            Eigen::Matrix3d rotation_mat;
//            rotation_mat = Eigen::AngleAxisd(parameters_[kf_idx*9], Eigen::Vector3d::UnitX())
//                *Eigen::AngleAxisd(parameters_[kf_idx*9 + 1], Eigen::Vector3d::UnitY())
//                *Eigen::AngleAxisd(parameters_[kf_idx*9 + 2], Eigen::Vector3d::UnitZ());
            cv::Mat rotation_vec_i(3,1,CV_64F);
            cv::Mat rotation_mat_i(3,3,CV_64F);
            rotation_vec_i.at<double>(0) = parameters_[kf_idx*9];
            rotation_vec_i.at<double>(1) = parameters_[kf_idx*9 + 1];
            rotation_vec_i.at<double>(2) = parameters_[kf_idx*9 + 2];
            cv::Rodrigues(rotation_vec_i, rotation_mat_i);

            cv::Mat rotation_mat = rotation_mat_i.t();

            for (int i = 0; i < 3; i++){
                for (int j = 0; j < 3; j++){
                    pose(i,j) = rotation_mat.at<double>(i,j);
                }
            }

            Eigen::Matrix3d rotation_mat_eig_i(3,3);
            cv::cv2eigen(rotation_mat_i, rotation_mat_eig_i);
            Eigen::Vector3d translation_vec_i(parameters_[kf_idx*9 + 3], parameters_[kf_idx*9 + 4], parameters_[kf_idx*9 + 5]);
            Eigen::Vector3d translation_vec = -1 * rotation_mat_eig_i.transpose() * translation_vec_i;
            // translation
            for (int i = 0; i < 3; i++){
                pose(i,3) = translation_vec(i);
            }

            // ignore optimization on camera intrinsic

            pose(3,3) = 1;
            keyframes[kf_idx]->set_pose(pose);
        }

        // fill in point poses
        int starting_idx = keyframes.size() * 9;
        for (auto point : map_points){
            auto point_id = point->GetID();
            Eigen::Vector3d point_pos = Eigen::Vector3d::Zero(3);
            point_pos[0] = parameters_[starting_idx + point_id * 3];
            point_pos[1] = parameters_[starting_idx + point_id * 3 + 1];
            point_pos[2] = parameters_[starting_idx + point_id * 3 + 2];

            point->SetWorldPos(point_pos);
        }
    }


    SnavelyReprojectionError::SnavelyReprojectionError(double observed_x,
                                                       double observed_y,
                                                       CameraParameters cam) :
            observed_x(observed_x),
            observed_y(observed_y),
            camera_fx(cam.fx),
            camera_fy(cam.fy),
            camera_cx(cam.cx),
            camera_cy(cam.cy){

    }

    template<typename T>
    bool SnavelyReprojectionError::operator()(const T *const camera, const T *const point, T *residuals) const {
        // camera[0,1,2] are the angle-axis rotation.
        T p[3];
        ceres::AngleAxisRotatePoint(camera, point, p);

        // camera[3,4,5] are the translation.
        p[0] += camera[3];
        p[1] += camera[4];
        p[2] += camera[5];

        // Project 3D points to 2D
        T xp = p[0] / p[2];
        T yp = p[1] / p[2];

        // Apply second and fourth order radial distortion.
//        const T &l1 = camera[7];
//        const T &l2 = camera[8];
//        T r2 = xp * xp + yp * yp;
//        T distortion = 1.0 + r2 * (l1 + l2 * r2);
        // DO NOT optimize the distortion here
        double distortion = 1;

        // Compute final projected point position.
        // const T &focal = camera[6];
        // DO NOT optimize the camera focus here
        T predicted_x = camera_fx * distortion * xp + camera_cx;
        T predicted_y = camera_fy * distortion * yp + camera_cy;

        // The error is the difference between the predicted and observed position.
        residuals[0] = predicted_x - observed_x;
        residuals[1] = predicted_y - observed_y;

        return true;
    }


    ceres::CostFunction* SnavelyReprojectionError::Create(const double observed_x, const double observed_y, const CameraParameters cam) {
        return (new ceres::AutoDiffCostFunction<SnavelyReprojectionError, 2, 9, 3>(
                new SnavelyReprojectionError(observed_x, observed_y,cam)));
    }
}