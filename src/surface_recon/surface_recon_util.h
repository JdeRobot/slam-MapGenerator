//
// Created by ernest on 18-7-7.
//

#ifndef SLAM_MAPGEN_SURFACE_RECON_UTIL_H
#define SLAM_MAPGEN_SURFACE_RECON_UTIL_H

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/poisson.h>
#include <pcl/io/vtk_io.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include "logging_util.h"

pcl::PointCloud<pcl::Normal>::Ptr pcl_compute_normal(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

pcl::PolygonMesh pcl_fast_surface_recon(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double mu, double maximumNearestNeighbors);
pcl::PolygonMesh pcl_poisson_recon(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);


//std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> pcl_ransac_plane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

void pcl_ransac_plane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

#endif //SLAM_MAPGEN_SURFACE_RECON_UTIL_H
