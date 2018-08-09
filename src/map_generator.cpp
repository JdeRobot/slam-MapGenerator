/***
 * Description:
 * Usage:
 *      ./map_generator  --gen-config
 */

#include <iostream>
#include <opencv2/core/persistence.hpp>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <pcl/io/pcd_io.h>
#include <thread>

#include "Camera.h"
#include "NodeConfig.h"
#include "logging_util.h"
#include "Map.h"
#include "Config.h"
#include "JdeRobotIO.h"
#include "logging_util.h"

// Loop Closing
#include "LoopDetector.h"
#include "types.h"
#include "pose_graph_3d.h"

// Bundle Adjustment
#include "bundle_ajustment.h"

// Surface Reconstruction
#include "surface_recon_util.h"

// visualization
#include "MapDrawer.h"
#include "Viewer.h"


using namespace MapGen;

// ==============================================================
// Helper functions
// ==============================================================

void init_config(NodeConfig& config){
    config.add_namespace("Common");
    config.add_namespace("LoopClosure");
    config.add_namespace("GlobalBA");
    config.add_namespace("SurfaceRecon");
    config.add_namespace("ReconFastTriangulation");
    config.add_namespace("PlaneRANSAC");

    config.add_param("Common", "img_dir", "string");
    config.add_param("Common", "trajectory", "string");
    config.add_param("Common", "enableLoopClosure", "double");
    config.add_param("Common", "enableBA", "double");
    config.add_param("Common", "enableSurfaceRecon", "double");

    config.add_param("LoopClosure", "Vocabulary", "string");
    config.add_param("LoopClosure", "loop_detection_threshold", "double");

    config.add_param("SurfaceRecon", "reconMethod", "string");

    config.add_param("ReconFastTriangulation", "mu", "double");
    config.add_param("ReconFastTriangulation", "maximumNearestNeighbors", "double");
    config.add_param("ReconFastTriangulation", "searchRadius", "double");
    config.add_param("PlaneRANSAC", "minPreserveRatio", "double");

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

// TODO (check IO works fine after integration)
void loop_closing(Map& map, Camera& camera, NodeConfig& config){
    LOG_INFO << "==============================Loop Closure================================" << std::endl;
    // Get Config
    std::string img_dir = config.get_string_param("Common", "img_dir");
    std::string vocabulary = config.get_string_param("LoopClosure", "Vocabulary");
    double threshold = config.get_double_param("LoopClosure", "loop_detection_threshold");

    // detect loops
    MapGen::LoopDetector detector(map,img_dir, vocabulary, threshold);

    // print out the loop detection pair
    auto closing_pairs = detector.getLoopClosingPairs();
    for (auto p : closing_pairs){
        LOG_INFO << "detected loop closing pair: " << p.first->GetFilename() << " & "
                 << p.second->GetFilename() << std::endl;
    }
//    LOG_INFO << "======================================================" << std::endl;
//    for (auto p : closing_pairs){
//        LOG_INFO << "detected loop closing pair: " << p.first->GetId() << " & "
//                 << p.second->GetId() << std::endl;
//    }
//    LOG_INFO << "======================================================" << std::endl;

    LOG_INFO << "Detection Completed, start optimization" << std::endl;


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
//    MapGen::JdeRobotIO::saveTrajectory(map,camera,"poses_original.yaml");
//    LOG_INFO << "original poses saved to poses_original.txt. " << std::endl;

    // optimize the pose graph
    ceres::Problem problem;
    MapGen::BuildOptimizationProblem(constraints, &poses, &problem);

    MapGen::SolveOptimizationProblem(&problem);


    // update the Keyframe pose (from the optimizer)
    for (auto frame : kfs){
        Eigen::Vector3d p = poses.at(frame->GetId()).p;
        Eigen::Matrix3d r = Eigen::Matrix3d(poses.at(frame->GetId()).q);

        // the frame pose
        Eigen::Matrix4d t = Eigen::Matrix4d::Identity();
        for (int i = 0; i < 3; i++){
            t(i,3) = p(i);
        }
        for (int i = 0; i < 3; i++){
            for (int j = 0; j < 3; j++){
                t(i,j) = r(i,j);
            }
        }
        frame->set_pose(t);
    }
    LOG_INFO << "======================================================" << std::endl;
}


// TODO: need change the IO
int bundle_ajustment(Map& map, const Camera& cam){
//    google::InitGoogleLogging(argv[0]);
//    if (argc != 2) {
//        std::cerr << "usage: simple_bundle_adjuster <bal_problem>\n";
//        return 1;
//    }
//
//    BALProblem bal_problem;
//    if (!bal_problem.LoadFile(argv[1])) {
//        std::cerr << "ERROR: unable to open file " << argv[1] << "\n";
//        return 1;
//    }
    BALProblem bal_problem;
    bal_problem.LoadFromMap(map, cam);

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
                                                 observations[2 * i + 1],
                                                 cam.get_camera_params());
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
    options.max_num_iterations = 2000;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.FullReport() << "\n";

    // save back to the map
    bal_problem.SaveToMap(map, cam);

    return 0;

}

// TODO: under dev (RANSAC is kind of working)
pcl::PolygonMeshPtr surface_recon(Map& map, Camera& cam, NodeConfig& config){
    if (config.get_string_param("SurfaceRecon","reconMethod") == "FastTriangulation"){
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = map.GetPC();

        double mu = config.get_double_param("ReconFastTriangulation", "mu");
        double maximumNearestNeighbors = config.get_double_param("ReconFastTriangulation", "maximumNearestNeighbors");
        double searchRadius = config.get_double_param("ReconFastTriangulation", "searchRadius");

        // TODO: for debugging only
        LOG_INFO << "Parameter mu: " << mu << std::endl;
        LOG_INFO << "Parameter maximumNearestNeighbors" << maximumNearestNeighbors << std::endl;

        auto triangles = pcl_fast_surface_recon(cloud, mu, maximumNearestNeighbors, searchRadius);

        // save the polygon
        pcl::io::saveVTKFile("mesh.vtk",triangles);
    }
    else if (config.get_string_param("SurfaceRecon","reconMethod") == "PlaneRANSAC"){
        // use RANSAC to remove outliners
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = map.GetPC();

        double minPreserveRatio = config.get_double_param("PlaneRANSAC","minPreserveRatio");
        auto clouds = pcl_ransac_plane(cloud, minPreserveRatio);

        // write the pointcloud to disk
//        pcl::PCDWriter writer;
//        for (int i = 0; i < clouds.size(); i++){
//            std::stringstream ss;
//            ss << "table_scene_lms400_plane_" << i << ".xyz";
//            writer.write<pcl::PointXYZ> (ss.str (), *clouds[i], false);
//        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_full(new pcl::PointCloud<pcl::PointXYZ>);
        for (int i = 0; i < clouds.size(); i++){
            *cloud_full = *cloud_full + *clouds[i];
        }
//        writer.write<pcl::PointXYZ> ("cloud_full.xyz", *cloud_full, false);


        // run surface reconstruction again to get the mesh
        double mu = config.get_double_param("ReconFastTriangulation", "mu");
        double maximumNearestNeighbors = config.get_double_param("ReconFastTriangulation", "maximumNearestNeighbors");
        double searchRadius = config.get_double_param("ReconFastTriangulation", "searchRadius");

        // individual reconstruction on each plane
        std::vector<pcl::PolygonMesh> meshes;
        for (int i = 0; i < clouds.size(); i++) {
            auto triangles = pcl_fast_surface_recon(clouds[i], mu, maximumNearestNeighbors, searchRadius);
            meshes.push_back(triangles);
        }

        // combine those planes
        pcl::PolygonMeshPtr triangles_full(new pcl::PolygonMesh);
        *triangles_full = concatenate_polygon_mesh(meshes);
        return triangles_full;
        // save the polygon
//        pcl::io::saveVTKFile("mesh.vtk",triangles_full);
    }
    else{
        LOG_ERROR << "Reconstruction method " << config.get_string_param("SurfaceRecon","reconMethod") << " not implemented." << std::endl;
        throw std::runtime_error("Reconstruction method not implemented");
    }
}


int show_result_user_interface(Map& map, NodeConfig * config, pcl::PolygonMeshPtr triangles){
    // Create user interface
    MapGen::MapDrawer * mdrawer = new MapGen::MapDrawer(&map, config, triangles);

    MapGen::Viewer* viewer = nullptr;
    std::thread* tviewer = nullptr;

    viewer = new MapGen::Viewer(mdrawer);
    tviewer = new std::thread(&MapGen::Viewer::Run, viewer);

    // Wait until threads start
    usleep(1000*1e3);

    // Main loop
    while (!viewer->isFinished()) {
        // Wait
        usleep(30*1e3);
    }

    viewer->RequestFinish();
    while (!viewer->isFinished())
        usleep(5000);

    tviewer->join();

    return 0;
}

int main(int argc, const char * argv[]){
    // deal with running parameters
    for (int i = 1; i < argc; i++){
        if (strcmp(argv[i], "--print-config") == 0){
            print_example_config();
            return 0;
        }
        else if (strcmp(argv[i], "--play") == 0){
            auto triangles = pcl_polygonmesh_playground();
            pcl::io::saveVTKFile("mesh.vtk",triangles);
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

    google::InitGoogleLogging(argv[0]);

    // read in node configuration
    NodeConfig config;
    init_config(config);
    config.read_from_file(argv[1]);

    // read in the map
    Map map;
    Camera cam;
    Config::ReadParameters(config.get_string_param("Common", "trajectory"), map, cam);

    // Loop Closure
    if (config.get_double_param("Common", "enableLoopClosure")){
        loop_closing(map,cam,config);
        // save trajectory for demo
        JdeRobotIO::saveTrajectory(map,cam,"pose_optimized.yaml");
    }

    // Global BA
    if (config.get_double_param("Common","enableBA") == 1){
        JdeRobotIO::saveTrajectory(map,cam,"BA_before_optimized.yaml");
        bundle_ajustment(map,cam);
        JdeRobotIO::saveTrajectory(map,cam,"BA_optimized.yaml");
    }

    // Surface Reconstruction
    pcl::PolygonMeshPtr triangles = nullptr;
    if (config.get_double_param("Common","enableSurfaceRecon") == 1){
        triangles = surface_recon(map,cam,config);;
    }

    show_result_user_interface(map, &config, triangles);

    return 0;
}