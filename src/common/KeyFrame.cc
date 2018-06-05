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

#include "KeyFrame.h"

using std::mutex;
using std::unique_lock;
using std::vector;

namespace MapGen {


KeyFrame::KeyFrame(int id, std::string filename, const vector<double> &pose):
    id_(id), filename_(filename) {
    assert(pose.size() == 7);

    // Get translation and rotation
    Eigen::Quaterniond q(pose[0], pose[1], pose[2], pose[3]);
    Eigen::Vector3d p(pose[4], pose[5], pose[6]);

    pose_.setIdentity();
    pose_.block<3, 3>(0, 0) = q.toRotationMatrix();
    pose_.block<3, 1>(0, 3) = p;
}

int KeyFrame::GetId() {
    return id_;
}

const std::string KeyFrame::GetFilename() {
    return filename_;
}

Eigen::Matrix4d KeyFrame::GetPose() {
    unique_lock<mutex> lock(mutexPose_);
    return pose_;
}

Eigen::Matrix3d KeyFrame::GetRotation() {
  unique_lock<mutex> lock(mutexPose_);
  return pose_.block<3, 3>(0, 0);
}

Eigen::Vector3d KeyFrame::GetTranslation() {
  unique_lock<mutex> lock(mutexPose_);
  return pose_.block<3, 1>(0, 3);
}

void KeyFrame::AddConnection(KeyFrame *kf, const int weight) {
    unique_lock<mutex> lock(mutexConnections_);
    connectedKeyFrames_[kf] = weight;
}

vector<KeyFrame*> KeyFrame::GetConnectedByWeight(const int &w) {
    vector<KeyFrame*> connections;

    unique_lock<mutex> lock(mutexConnections_);

    // Save KeyFrames greater than weight
    for (auto it=connectedKeyFrames_.begin(); it!=connectedKeyFrames_.end(); it++) {
        if (it->second >= w)
            connections.push_back(it->first);
    }

    return connections;
}

void KeyFrame::AddObservation(MapPoint* mp, const std::vector<double> &pixel) {
    assert(pixel.size() == 2);
    Eigen::Vector2d kp(pixel[0], pixel[1]);

    unique_lock<mutex> lock(mutexFeatures_);
    mapPoints_[mp] = kp;
    mp->AddObservation(this);
}

void KeyFrame::EraseObservation(MapPoint* mp) {
    unique_lock<mutex> lock(mutexFeatures_);
    if(mapPoints_.count(mp)) {
        mapPoints_.erase(mp);
    }
}

void KeyFrame::UpdateConnections() {
    std::map<KeyFrame*, int> KFcounter;

    vector<MapPoint*> mapPoints;

    {
        unique_lock<mutex> lockMPs(mutexFeatures_);
        for (auto it=mapPoints_.begin(); it!=mapPoints_.end(); it++)
            mapPoints.push_back(it->first);
    }

    // For each map point in keyframe: check in which other keyframes they are seen and
    // increase counter for those keyframes
    for (vector<MapPoint*>::iterator vit=mapPoints.begin(), vend=mapPoints.end(); vit != vend; vit++) {
        MapPoint* mp = *vit;

        if (!mp)
            continue;

        std::set<KeyFrame*> observations = mp->GetObservations();
        for (auto mit=observations.begin(), mend=observations.end(); mit != mend; mit++) {
            KeyFrame* kf = *mit;

            if (kf->id_ == id_)
                continue;

            KFcounter[kf]++;
        }
    }

    // This should not happen
    if (KFcounter.empty())
        return;

    // If the counter is greater than threshold add connection
    // In case no keyframe counter is over threshold add the one with maximum counter
    int nmax = 0;
    KeyFrame* kf_max = nullptr;
    int th = 15;

    vector<std::pair<int,KeyFrame*> > vPairs;
    vPairs.reserve(KFcounter.size());
    for (auto mit=KFcounter.begin(), mend=KFcounter.end(); mit != mend; mit++) {
        if (mit->second > nmax) {
            nmax = mit->second;
            kf_max = mit->first;
        }
        if (mit->second >= th) {
            vPairs.push_back(std::make_pair(mit->second, mit->first));
            (mit->first)->AddConnection(this, mit->second);
        }
    }

    if (vPairs.empty()) {
        vPairs.push_back(std::make_pair(nmax,kf_max));
        kf_max->AddConnection(this, nmax);
    }

    // Add connections
    for (size_t i = 0; i < vPairs.size(); i++)
        AddConnection(vPairs[i].second, vPairs[i].first);
}

void KeyFrame::EraseConnection(KeyFrame* kf) {
    unique_lock<mutex> lock(mutexConnections_);
    if (connectedKeyFrames_.count(kf))
        connectedKeyFrames_.erase(kf);
}

void KeyFrame::setBowVector(const DBoW2::BowVector &bow_vector) {
    bow_vector_ = bow_vector;
}

DBoW2::BowVector KeyFrame::getBowVector() {
    return bow_vector_;
}

}  // namespace SLAM_VIEWER
