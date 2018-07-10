//
// Created by ernest on 18-6-28.
//

#ifndef SLAM_MAPGEN_OPTIMIZER_H
#define SLAM_MAPGEN_OPTIMIZER_H

//#include <g2o/core/sparse_optimizer.h>
//#include <g2o/core/block_solver.h>
//#include <g2o/solvers/eigen/linear_solver_eigen.h>
//#include <g2o/core/optimization_algorithm_levenberg.h>

#include "Map.h"
#include "LoopConnection.h"

//using namespace g2o;

namespace MapGen {

    class Optimizer {
    public:
        // TODO: maybe initialize the scaling_factor with several observation if you need
        Optimizer(std::vector<KeyFrame *> frames, cv::Mat K);

        // the nodes should be added sequentially
        void add_loop_closing(KeyFrame * frame_a, KeyFrame * frame_b);

    private:
        std::vector<LoopConnection> loop_connections_;

        // the scaling factor used to rescale the relative pose (Sim3) solved by OpenCV
        // so that it could meet with the ground truth
        // should be initialized with several observation before loop closing
        double scaling_factor_;
    };
}

#endif //SLAM_MAPGEN_OPTIMIZER_H
