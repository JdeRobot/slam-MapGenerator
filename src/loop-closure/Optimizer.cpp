//
// Created by ernest on 18-6-28.
//

#include "Optimizer.h"


namespace MapGen{
    // ORB2 somehow extend g2o a lot, we are going to use ceres solver, it's more simple
//    Optimizer::Optimizer(Map &map, std::vector<LoopConnection> &loop_connections) {
//        g2o::SparseOptimizer optimizer;
//        optimizer.setVerbose(false);
//        g2o::BlockSolver_7_3::LinearSolverType * linearSolver =
//                new g2o::LinearSolverEigen<g2o::BlockSolver_7_3::PoseMatrixType>();
//        g2o::BlockSolver_7_3 * solver_ptr= new g2o::BlockSolver_7_3(linearSolver);
//        g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
//
//        solver->setUserLambdaInit(1e-16);
//        optimizer.setAlgorithm(solver);
//
//        // TODO: add frame-to-frame connection to the optimization graph
//
//
//        // TODO: add loop-closing observation to the optimization graph
//    }

    Optimizer::Optimizer(std::vector<KeyFrame *> frames, cv::Mat K) :loop_connections_(), scaling_factor_(0.0) {
        if (frames.size() < 2){
            LOG_ERROR << "Unable to initialize the optimizer. At least 2 keyframes are required.";
        }

        // calculate the scaling factor
        // scaling_factor is set to 1 so that we can determine the scaling factor later
        std::vector<double> ts;
        LOG_INFO << "ts: " << std::endl;
        for (int i = 0; i < 10; i++){
            // use OpenCV to get the frame-to-frame position
            LoopConnection tmp_connection(frames[i], frames[i+1], K, 1);
            double tmp_1 = tmp_connection.get_translation().at<double>(0);

            auto P1 = frames[i]->GetPose();
            auto P2 = frames[i+1]->GetPose();
            auto T = P1.inverse() * P2;

            double tmp_2 = T(0,3);


            // use input data to get the scaling factor
            ts.push_back(tmp_1/tmp_2);

            LOG_INFO << tmp_1/tmp_2 << std::endl;
        }

    }

}