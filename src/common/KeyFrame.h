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

#ifndef SLAM_MAPGEN_KEYFRAME_H
#define SLAM_MAPGEN_KEYFRAME_H

#include <map>
#include <mutex>
#include <BowVector.h>
#include "MapPoint.h"

namespace MapGen {

class MapPoint;

class KeyFrame {
 public:
    KeyFrame(int id, std::string filename, const std::vector<double> &pose);

    int GetId();
    const std::string GetFilename();

    // Pose functions
    Eigen::Matrix4d GetPose();
    Eigen::Matrix3d GetRotation();
    Eigen::Vector3d GetTranslation();

    // Covisibility graph functions
    void AddConnection(KeyFrame* kf, const int weight);
    void EraseConnection(KeyFrame* kf);
    void UpdateConnections();
    std::vector<KeyFrame*> GetConnectedByWeight(const int &w);

    // MapPoint observation functions
    void AddObservation(MapPoint* mp, const std::vector<double> &pixel);
    void EraseObservation(MapPoint* mp);

    void setBowVector(const DBoW2::BowVector& bow_vector);
    DBoW2::BowVector getBowVector();

 private:
    // KeyFrame id
    int id_;

    // Source filename (if exists)
    std::string filename_;

    // SE3 Pose
    Eigen::Matrix4d pose_;

    // MapPoints and associated pixels
    std::map<MapPoint*, Eigen::Vector2d> mapPoints_;

    // Connected KeyFrames and weights
    std::map<KeyFrame*, int> connectedKeyFrames_;

    std::mutex mutexPose_;
    std::mutex mutexConnections_;
    std::mutex mutexFeatures_;

    // BoW Vector
    DBoW2::BowVector bow_vector_;

 public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace SLAM_MAPGEN

#endif  // SLAM_MAPGEN_KEYFRAME_H
