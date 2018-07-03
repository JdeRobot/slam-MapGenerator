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

    Optimizer::Optimizer(Map &map, std::vector<LoopConnection> &loop_connections) {

    }

}