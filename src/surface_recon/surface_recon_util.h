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
#include <pcl/io/vtk_io.h>

pcl::PolygonMesh pcl_fast_surface_recon(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);


#endif //SLAM_MAPGEN_SURFACE_RECON_UTIL_H
