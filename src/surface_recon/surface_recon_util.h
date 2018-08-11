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

pcl::PolygonMesh pcl_fast_surface_recon(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double mu,
        double maximumNearestNeighbors, double searchRadius);

pcl::PolygonMesh pcl_poisson_recon(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

pcl::PolygonMesh pcl_polygonmesh_playground();

//std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> pcl_ransac_plane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

// @Params:
//      min_valid_points: the ratio of points to keep in the output (reconstructed surfaces)
std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> pcl_ransac_plane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
        double minPreserveRatio);

// Generate a rectangular based on the four corners
// @Params:
//      corners:            the four corners to define a rectangular
pcl::PolygonMesh build_rect_mesh(std::vector<pcl::PointXYZ> corners);

pcl::PolygonMesh concatenate_polygon_mesh(std::vector<pcl::PolygonMesh> input_meshes);

#endif //SLAM_MAPGEN_SURFACE_RECON_UTIL_H
