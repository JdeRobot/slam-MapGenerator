/***
 * Description:
 * Usage:
 *      ./map_generator  --gen-config
 */

#include <iostream>
#include <opencv2/core/persistence.hpp>
#include "NodeConfig.h"
#include "logging_util.h"

using namespace MapGen;


void print_example_config(){
    LOG_INFO << "Generating example config to example.yaml" << std::endl;

    NodeConfig config;

    config.add_namespace("Common");
    config.add_namespace("LoopClosure");
    config.add_namespace("GlobalBA");
    config.add_namespace("SurfaceRecon");

    config.add_param("Common", "img_dir", "string");
    config.add_param("Common", "trajectory", "string");
    config.add_param("Common", "EnableLoopClosure", "double");
    config.add_param("LoopClosure", "Vocabulary", "string");
    config.add_param("LoopClosure", "loop_detection_threshold", "double");
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

int main(int argc, const char * argv[]){
    // deal with running parameters
    for (int i = 1; i < argc; i++){
        if (strcmp(argv[i], "--print-config") == 0){
            print_example_config();
        }
    }

    if (argc == 3){
        LOG_ERROR << "Invalid Usage." << std::endl;
    }



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