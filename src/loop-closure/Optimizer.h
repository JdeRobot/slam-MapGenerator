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
        Optimizer(Map& map, std::vector<LoopConnection>& loop_connections);
    };

}

#endif //SLAM_MAPGEN_OPTIMIZER_H
