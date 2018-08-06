/**
 *
 *  Copyright (C) 2018 Eduardo Perdices <eperdices at gsyc dot es>
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

#ifndef SLAM_VIEWER_MAPDRAWER_H
#define SLAM_VIEWER_MAPDRAWER_H

#include <mutex>
#include <pangolin/pangolin.h>
#include <Eigen/Dense>
#include <opencv2/core/core.hpp>
#include <pcl/PolygonMesh.h>
#include <pcl/conversions.h>
#include "Map.h"

namespace MapGen {

class MapDrawer {
 public:
    MapDrawer(Map *map);

    MapDrawer(Map *map, pcl::PolygonMeshPtr mesh);

    void DrawMapPoints();
    void DrawKeyFrames(const bool bDrawKF, const bool bDrawGraph);
    void DrawSurface();


    // ==================== Helper Functions =============================
    void DrawTriangle(const Eigen::Vector3d& v1, const Eigen::Vector3d& v2, const Eigen::Vector3d& v3, bool draw_border = false);
    void DrawTriangle(pcl::PointXYZ pt1, pcl::PointXYZ pt2, pcl::PointXYZ pt3, bool draw_border);

 private:
    Map * map_;
    pcl::PolygonMeshPtr mesh_;

 public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace SLAM_VIEWER

#endif  // SLAM_VIEWER_MAPDRAWER_H
