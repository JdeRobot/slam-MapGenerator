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
    n.setKSearch(5);
    n.compute(*normals);
//* normals should not contain the point normals + surface curvatures

    LOG_INFO << "Normal Estimation Completed." << std::endl;

    return normals;
}

pcl::PolygonMesh pcl_fast_surface_recon(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
        double mu, double maximumNearestNeighbors, double searchRadius) {

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
    gp3.setSearchRadius(searchRadius);

// Set typical values for the parameters
    gp3.setMu(mu);
    gp3.setMaximumNearestNeighbors((int)maximumNearestNeighbors);
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

pcl::PolygonMesh pcl_polygonmesh_playground(){
    pcl::PolygonMesh mesh;

    // create vertex
    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.push_back(pcl::PointXYZ(0,0,0));
    cloud.push_back(pcl::PointXYZ(0,1,0));
    cloud.push_back(pcl::PointXYZ(1,0,0));
    cloud.push_back(pcl::PointXYZ(1,1,0));

    pcl::PCLPointCloud2 cloud2;
    pcl::toPCLPointCloud2(cloud,cloud2);
    mesh.cloud = cloud2;

    // create polygon
    std::vector<pcl::Vertices> vertices_vector;

    pcl::Vertices v1, v2;
    v1.vertices.push_back(0);
    v1.vertices.push_back(1);
    v1.vertices.push_back(2);

    v2.vertices.push_back(1);
    v2.vertices.push_back(2);
    v2.vertices.push_back(3);

    vertices_vector.push_back(v1);
    vertices_vector.push_back(v2);

    mesh.polygons = vertices_vector;

    return mesh;
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


std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> pcl_ransac_plane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double min_preserve_ratio) {

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds;

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

    int nr_points = (int) cloud->points.size();

    // While 30% of the original cloud is still there
    while (cloud->points.size() > (1-min_preserve_ratio) * nr_points) {
        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud(cloud);
        seg.segment(*inliers, *coefficients);
        if (inliers->indices.size() == 0) {
            LOG_ERROR << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
        }

        // Extract the inliers
        extract.setInputCloud(cloud);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*cloud_p);
        LOG_INFO << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height
                  << " data points." << std::endl;
        LOG_INFO << "Model Coefficients: ";
        for (int i = 0 ; i < coefficients.get()->values.size(); i++){
            LOG_INFO << coefficients.get()->values[i] << "  ";
        }
        LOG_INFO << std::endl;

        // Save the pointcloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_to_save(new pcl::PointCloud<pcl::PointXYZ>);
        *cloud_to_save = *cloud_p;
        clouds.push_back(cloud_to_save);

        // Create the filtering object
        extract.setNegative (true);
        extract.filter (*cloud_f);
        cloud.swap (cloud_f);
    }

    return clouds;
}

pcl::PolygonMesh build_rect_mesh(std::vector<pcl::PointXYZ> corners){
    if (corners.size() != 4){
        throw std::runtime_error("Error input for build_rect_mesh(), only 4 corners are acceptable.");
    }

    pcl::PolygonMesh mesh;

    // create vertex
    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.push_back(pcl::PointXYZ(0,0,0));
    cloud.push_back(pcl::PointXYZ(0,1,0));
    cloud.push_back(pcl::PointXYZ(1,0,0));
    cloud.push_back(pcl::PointXYZ(1,1,0));

    pcl::PCLPointCloud2 cloud2;
    pcl::toPCLPointCloud2(cloud,cloud2);
    mesh.cloud = cloud2;

    // create polygon
    std::vector<pcl::Vertices> vertices_vector;

    pcl::Vertices v1, v2;
    v1.vertices.push_back(0);
    v1.vertices.push_back(1);
    v1.vertices.push_back(2);

    v2.vertices.push_back(1);
    v2.vertices.push_back(2);
    v2.vertices.push_back(3);

    vertices_vector.push_back(v1);
    vertices_vector.push_back(v2);

    mesh.polygons = vertices_vector;

    return mesh;
}


pcl::PolygonMesh concatenate_polygon_mesh(std::vector<pcl::PolygonMesh> input_meshes){
    pcl::PolygonMesh output_mesh;
    pcl::PointCloud<pcl::PointXYZ> output_cloud;

    int point_idx_offset = 0;
    for (auto mesh : input_meshes){
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromPCLPointCloud2(mesh.cloud,cloud);

        // add vertices to the new mesh
        output_cloud = output_cloud + cloud;

        // add polygon to the new mesh
        for (pcl::Vertices polygon_old : mesh.polygons){

            pcl::Vertices polygon_new;

            polygon_new.vertices.push_back(polygon_old.vertices[0] + point_idx_offset);
            polygon_new.vertices.push_back(polygon_old.vertices[1] + point_idx_offset);
            polygon_new.vertices.push_back(polygon_old.vertices[2] + point_idx_offset);

            output_mesh.polygons.push_back(polygon_new);
        }

        // update the mesh offset
        point_idx_offset += cloud.points.size();
    }

    // set the output mesh's pointcloud
    pcl::toPCLPointCloud2(output_cloud,output_mesh.cloud);
    return output_mesh;
}