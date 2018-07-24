/***
 * Description:
 * Usage:
 *      ./map_generator  --gen-config
 */

#include <iostream>
#include <opencv2/core/persistence.hpp>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include "NodeConfig.h"
#include "logging_util.h"
#include "bundleajustment.h"

using namespace MapGen;

// ==============================================================
// Helper functions
// ==============================================================

void init_config(NodeConfig& config){
    config.add_namespace("Common");
    config.add_namespace("LoopClosure");
    config.add_namespace("GlobalBA");
    config.add_namespace("SurfaceRecon");

    config.add_param("Common", "img_dir", "string");
    config.add_param("Common", "trajectory", "string");
    config.add_param("Common", "EnableLoopClosure", "double");
    config.add_param("LoopClosure", "Vocabulary", "string");
    config.add_param("LoopClosure", "loop_detection_threshold", "double");
}

// ==============================================================
// Features
// ==============================================================

void print_example_config(){
    LOG_INFO << "Generating example config to example.yaml" << std::endl;

    NodeConfig config;

    init_config(config);
    // config.add_param("Common", "img_dir", "string");

    // set default params
    config.set_string_param("Common","img_dir", "/home/ernest/SLAM/datasets/sequence_43/undistorted_images/");
    config.set_string_param("Common","trajectory", "/home/ernest/SLAM/datasets/trajectory/trajectory_noloop.yaml");
    config.set_double_param("Common","EnableLoopClosure", 1);
    config.set_string_param("LoopClosure","Vocabulary", "/home/ernest/SLAM/slam-MapGenerator/Vocabulary/ORBvoc.tar.gz");
    config.set_double_param("LoopClosure","loop_detection_threshold", 0.575);

    config.dump("example.yaml");

    LOG_INFO << "Example Config generated and saved to example.yaml" << std::endl;
}



// TODO: need change the IO
int bundle_ajustment(int argc, const char * argv[]){
    google::InitGoogleLogging(argv[0]);
    if (argc != 2) {
        std::cerr << "usage: simple_bundle_adjuster <bal_problem>\n";
        return 1;
    }

    BALProblem bal_problem;
    if (!bal_problem.LoadFile(argv[1])) {
        std::cerr << "ERROR: unable to open file " << argv[1] << "\n";
        return 1;
    }

    const double* observations = bal_problem.observations();

    // Create residuals for each observation in the bundle adjustment problem. The
    // parameters for cameras and points are added automatically.
    ceres::Problem problem;
    for (int i = 0; i < bal_problem.num_observations(); ++i) {
        // Each Residual block takes a point and a camera as input and outputs a 2
        // dimensional residual. Internally, the cost function stores the observed
        // image location and compares the reprojection against the observation.

        ceres::CostFunction* cost_function =
                SnavelyReprojectionError::Create(observations[2 * i + 0],
                                                 observations[2 * i + 1]);
        problem.AddResidualBlock(cost_function,
                                 NULL /* squared loss */,
                                 bal_problem.mutable_camera_for_observation(i),
                                 bal_problem.mutable_point_for_observation(i));
    }

    // Make Ceres automatically detect the bundle structure. Note that the
    // standard solver, SPARSE_NORMAL_CHOLESKY, also works fine but it is slower
    // for standard bundle adjustment problems.
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.minimizer_progress_to_stdout = true;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.FullReport() << "\n";
    return 0;

}



int main(int argc, const char * argv[]){
    // deal with running parameters
    for (int i = 1; i < argc; i++){
        if (strcmp(argv[i], "--print-config") == 0){
            print_example_config();
            return 0;
        }
        else{
            // raise error if it is not the last argument
            if (i != argc - 1) {
                LOG_ERROR << "Unrecognized argument: " << argv[i] << std::endl;
                return 1;
            }
        }
    }

    // The main program takes 1 parameter1 only
    if (argc != 2){
        LOG_ERROR << "Invalid Usage." << std::endl;
    }


    NodeConfig config;
    init_config(config);
    config.read_from_file(argv[1]);


//    NodeConfig config;
//    config.add_namespace("A");
//    config.add_param("A", "A1", "double");
//    config.add_param("A", "A2", "double");
//
//    config.set_double_param("A", "A1", 1);
//    config.set_double_param("A", "A2", 2);
//
//    config.dump("example.yaml");

    return 0;
}