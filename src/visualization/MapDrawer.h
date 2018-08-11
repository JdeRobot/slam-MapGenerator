/**
 *
 *  Copyright (C) 2018 Jianxiong Cai <caijx AT shanghaitech.edu.cn>
 *
 *  The following code is a derivative work of the code from the slam-viewer project,
 *  which is licensed under the GNU Public License, version 3. This code therefore
 *  is also licensed under the terms of the GNU Public License, version 3.
 *  For more information see <https://github.com/JdeRobot/slam-viewer>.
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
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <pcl/PolygonMesh.h>
#include <pcl/conversions.h>
#include <cmath>
#include <fstream>
#include "Map.h"
#include "MapPoint.h"
#include "Config.h"
#include "SOIL.h"
#include "NodeConfig.h"

namespace MapGen {

class MapDrawer {
 public:
    MapDrawer(Map *map, NodeConfig * config);

    MapDrawer(Map *map, NodeConfig * config, pcl::PolygonMeshPtr mesh);

    void DrawMapPoints();
    void DrawKeyFrames(const bool bDrawKF, const bool bDrawGraph);
    void DrawSurface();


    // ==================== Helper Functions =============================
    void DrawTriangle(const Eigen::Vector3d& v1, const Eigen::Vector3d& v2, const Eigen::Vector3d& v3, bool draw_border = false);
    void DrawTriangle(pcl::PointXYZ pt1, pcl::PointXYZ pt2, pcl::PointXYZ pt3, bool draw_border);

    // the texture will only refresh once
    void DrawTriangleTexture(std::vector<std::pair<pcl::PointXYZ, MapPoint *>> points, bool draw_border);
    bool DrawTriangleTexture(int polygon_idx, bool draw_boarder);

    // Get the set of observations that the 3 points must be observed in the same image.
    std::vector<std::pair<KeyFrame *, Eigen::Vector2d>> GetObservations(std::vector<MapPoint *> points);

    inline MapPoint * SearchNearest(pcl::PointXYZ point);

    void BuildCorrespondence();

 private:
    Map * map_;
    pcl::PolygonMeshPtr mesh_;
    NodeConfig * config_;

    //
    // cached variables to accelerate the GUI
    //
    // points (index: point index), establish on startup
    std::map<int, MapPoint *> pt_correspondence_;
    // index: polygon index in the mesh
    std::map<int, std::vector<std::pair<KeyFrame *, Eigen::Vector2d>>> cached_observations_;
    // the texture
    std::map<KeyFrame *, GLuint> surf_textures_;
    // width first
    std::map<KeyFrame *, std::pair<int, int>> surf_shapes_;

    std::ofstream debug_file;

 public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace SLAM_VIEWER

#endif  // SLAM_VIEWER_MAPDRAWER_H
