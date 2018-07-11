/**
 *
 *  Copyright (C) 2018 Jianxiong Cai <caijx AT shanghaitech.edu.cn>
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
 */

#include <glog/logging.h>
#include "Config.h"
#include "NodeConfig.h"
#include "Map.h"
#include "LoopDetector.h"
#include "LoopConnection.h"
#include "logging_util.h"
#include "Camera.h"
#include "pose_graph_3d.h"
#include "types.h"
#include "JdeRobotIO.h"

using namespace MapGen;

int main (int argc, const char * argv[]){
    init_logging();

    if (argc != 2){
        LOG_ERROR << "Usage error" <<std::endl;
        return 1;
    }


    NodeConfig config(argv[1]);
    Map map;
    Camera camera;


    // read in the trajectory file
    if(!MapGen::Config::ReadParameters(config.get_trajectory(), map, camera)){
        LOG_ERROR << "fail to read the trajectory file at: " << config.get_trajectory() << std::endl;
        return 1;
    }

    // detect loops
    MapGen::LoopDetector detector(map,config.get_img_dir(), config.get_vocabulary(), config.get_threshold());

    // print out the loop detection pair
    auto closing_pairs = detector.getLoopClosingPairs();
    for (auto p : closing_pairs){
        LOG_INFO << "detected loop closing pair: " << p.first->GetFilename() << " & "
                                << p.second->GetFilename() << std::endl;
    }
    LOG_INFO << "======================================================" << std::endl;
    for (auto p : closing_pairs){
        LOG_INFO << "detected loop closing pair: " << p.first->GetId() << " & "
                 << p.second->GetId() << std::endl;
    }

    LOG_INFO << "Detection Completed" << std::endl;


    // try to close the loop
    // create a BFMatcher
//    cv::BFMatcher matcher;
//    vector<LoopConnection *> loop_connections;
//    for (auto p : closing_pairs){
//        loop_connections.push_back(new LoopConnection(p, camera.get_intrinsic_matrix()));
//    }

    google::InitGoogleLogging(argv[0]);
    MapGen::MapOfPoses poses;
    MapGen::VectorOfConstraints constraints;
    // add all vertex (keyframes)
    for (auto kf : map.GetAllKeyFrames()){
        poses.insert(std::pair<int,Pose3d>(kf->GetId(), Pose3d(kf->GetPose())));
    }

    // build constrains
    // sequential constrains
    auto kfs = map.GetAllKeyFrames();
    for (int i = 0; i < (kfs.size() - 1); i++){
        auto kf_1 = kfs[i];
        auto kf_2 = kfs[i+1];
        Eigen::Matrix4d t = kf_1->GetPose().inverse() * kf_2->GetPose();
        constraints.push_back(Constraint3d(kf_1->GetId(), kf_2->GetId(), t));
    }
    // loop closing constrains
    for (auto p : closing_pairs){
        auto kf_1 = p.first;
        auto kf_2 = p.second;
        // TODO: assume the two frames are really close such that their transform is I
        // TODO: need to have a PNP slover for solving the 3D pose
        Eigen::Matrix4d t = Eigen::Matrix4d::Identity();
        constraints.push_back(Constraint3d(kf_1->GetId(), kf_2->GetId(), t));
    }

    LOG_INFO << "Number of poses: " << poses.size() << '\n';
    LOG_INFO << "Number of constraints: " << constraints.size() << '\n';

    // save the original pose
    // MapGen::OutputPoses("poses_original.txt", poses);
    MapGen::JdeRobotIO::saveTrajectory(map,camera,"poses_original.yaml");
    LOG_INFO << "original poses saved to poses_original.txt. " << std::endl;

    // optimize the pose graph
    ceres::Problem problem;
    MapGen::BuildOptimizationProblem(constraints, &poses, &problem);

    bool result = MapGen::SolveOptimizationProblem(&problem);
    LOG_INFO << "Solving result: " << result << std::endl;

    // MapGen::OutputPoses("poses_optimized.txt", poses);
    MapGen::JdeRobotIO::saveTrajectory(map,camera,"poses_optimized.yaml");
    LOG_INFO << "optimized poses saved to poses_optimized.txt. " << std::endl;

//    Optimizer opt(map.GetAllKeyFrames(),camera);
//    for (auto p : closing_pairs){
//        opt.add_loop_closing(p.first, p.second);
//    }

    // update the Keyframe pose (from the optimizer)
//    for (auto frame : kfs){
//        auto pose_corrected = poses[frame->GetId()];
//    }

    return 0;
}