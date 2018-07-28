#include "surface_recon_util.h"


pcl::PointCloud<pcl::Normal>::Ptr pcl_compute_normal(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
    LOG_INFO << "Normal Estimation Start." << std::endl;
    // Normal estimation*
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);
    n.setInputCloud(cloud);
    n.setSearchMethod(tree);
    n.setKSearch(20);
    n.compute(*normals);
//* normals should not contain the point normals + surface curvatures

    LOG_INFO << "Normal Estimation Completed." << std::endl;

    return normals;
}

pcl::PolygonMesh pcl_fast_surface_recon(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double mu, double maximumNearestNeighbors) {
    // compute the normals
    auto normals = pcl_compute_normal(cloud);
    // Concatenate the XYZ and normal fields*
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);

//* cloud_with_normals = cloud + normals

    LOG_INFO << "Start surface reconstruction via fast triangulation" << std::endl;
// Create search tree*
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
    tree2->setInputCloud(cloud_with_normals);

// Initialize objects
    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
    pcl::PolygonMesh triangles;

// Set the maximum distance between connected points (maximum edge length)
    gp3.setSearchRadius(0.025);

// Set typical values for the parameters
    gp3.setMu(mu);
    gp3.setMaximumNearestNeighbors(maximumNearestNeighbors);
    gp3.setMaximumSurfaceAngle(M_PI / 4); // 45 degrees
    gp3.setMinimumAngle(M_PI / 18); // 10 degrees
    gp3.setMaximumAngle(2 * M_PI / 3); // 120 degrees
    gp3.setNormalConsistency(false);

// Get result
    gp3.setInputCloud(cloud_with_normals);
    gp3.setSearchMethod(tree2);
    gp3.reconstruct(triangles);

    LOG_INFO << "Reconstruction Completed." << std::endl;

// Additional vertex information
    std::vector<int> parts = gp3.getPartIDs();
    std::vector<int> states = gp3.getPointStates();

    return triangles;
}



pcl::PolygonMesh pcl_poisson_recon(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
    // compute the normals
    auto normals = pcl_compute_normal(cloud);
    // Concatenate the XYZ and normal fields*
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);

    LOG_INFO << "Start surface reconstruction via poisson reconstruction." << std::endl;
    // start triangulation
    pcl::PolygonMesh triangles;

    // use PCL libraries for reconstruction
    pcl::Poisson<pcl::PointNormal> poisson_recon;
    poisson_recon.setMinDepth(4);
    poisson_recon.setInputCloud(cloud_with_normals);
    poisson_recon.reconstruct(triangles);

    LOG_INFO << "Reconstruction Completed." << std::endl;

    return triangles;
}


//std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> pcl_ransac_plane(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in){
//
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(cloud_in);
//
//    while(1) {
//        std::vector<int> inliers;
//
//        pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr
//                model_p(new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(cloud));
//
//        pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_p);
//        ransac.setDistanceThreshold(.01);
//        ransac.computeModel();
//        ransac.getInliers(inliers);
//
//
//
//    }
//}


void pcl_ransac_plane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {

    // Write the downsampled version to disk
    pcl::PCDWriter writer;
    writer.write<pcl::PointXYZ> ("original.xyz", *cloud, false);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p (new pcl::PointCloud<pcl::PointXYZ>),
        cloud_f (new pcl::PointCloud<pcl::PointXYZ>);

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients(true);
    // Mandatory
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(1000);
    seg.setDistanceThreshold(0.02);

    // Create the filtering object
    pcl::ExtractIndices<pcl::PointXYZ> extract;

    int i = 0, nr_points = (int) cloud->points.size();

    // While 30% of the original cloud is still there
    while (cloud->points.size() > 0.3 * nr_points) {
        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud(cloud);
        seg.segment(*inliers, *coefficients);
        if (inliers->indices.size() == 0) {
            std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
        }

        // Extract the inliers
        extract.setInputCloud(cloud);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*cloud_p);
        std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height
                  << " data points." << std::endl;

        std::stringstream ss;
        ss << "table_scene_lms400_plane_" << i << ".xyz";
        writer.write<pcl::PointXYZ> (ss.str (), *cloud_p, false);

        // Create the filtering object
        extract.setNegative (true);
        extract.filter (*cloud_f);
        cloud.swap (cloud_f);
        i++;
    }
}