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

#include "MapDrawer.h"
#include "Config.h"

using std::vector;

namespace MapGen {

MapDrawer::MapDrawer(Map *map) {
    map_ = map;
}

void MapDrawer::DrawMapPoints() {
    const std::vector<MapPoint*> &points = map_->GetAllMapPoints();

    if (points.empty())
        return;

    glPointSize(Config::PointSize());
    glBegin(GL_POINTS);
    glColor3f(0.0, 0.0, 0.0);

    for (size_t i = 0, iend=points.size(); i < iend; i++) {
        Eigen::Vector3d pos = points[i]->GetWorldPos();
        glVertex3f(pos(0), pos(1), pos(2));
    }
    glEnd();
}

void MapDrawer::DrawKeyFrames(const bool bDrawKF, const bool bDrawGraph) {
    pangolin::OpenGlMatrix glmatrix;
    const float &w = Config::KeyFrameSize();
    const float h = w*0.75;
    const float z = w*0.6;

    const std::vector<KeyFrame*> &keyframes = map_->GetAllKeyFrames();

    if (bDrawKF) {
        double lwidth = Config::KeyFrameLineWidth();
        for (size_t i = 0; i < keyframes.size(); i++) {
            Eigen::Matrix4d Twc = keyframes[i]->GetPose();

            // Convert to cv::Mat
            cv::Mat Twc_cv(4, 4, CV_32F);
            for (int i = 0; i < 4; i++)
                for (int j = 0; j < 4; j++)
                    Twc_cv.at<float>(i, j) = Twc(j, i);

            glPushMatrix();

            glMultMatrixf(Twc_cv.ptr<GLfloat>(0));

            glLineWidth(lwidth);
            glColor3f(0.0f, 0.0f, 0.9f);
            glBegin(GL_LINES);
            glVertex3f(0, 0, 0);
            glVertex3f(w, h, z);
            glVertex3f(0, 0, 0);
            glVertex3f(w,-h, z);
            glVertex3f(0, 0, 0);
            glVertex3f(-w,-h, z);
            glVertex3f(0, 0, 0);
            glVertex3f(-w, h, z);

            glVertex3f(w, h, z);
            glVertex3f(w,-h, z);

            glVertex3f(-w, h, z);
            glVertex3f(-w,-h, z);

            glVertex3f(-w, h, z);
            glVertex3f(w, h, z);

            glVertex3f(-w,-h, z);
            glVertex3f(w,-h, z);
            glEnd();

            glPopMatrix();
        }
    }

    if (bDrawGraph) {
        glLineWidth(Config::GraphLineWidth());
        glColor4f(0.7f, 0.0f, 0.7f, 0.6f);
        glBegin(GL_LINES);

        for (size_t i = 0; i < keyframes.size(); i++) {
            // Covisibility Graph
            const vector<KeyFrame*> vCovKFs = keyframes[i]->GetConnectedByWeight(50);

            Eigen::Vector3d Ow = keyframes[i]->GetTranslation();
            if (!vCovKFs.empty()) {
                for (vector<KeyFrame*>::const_iterator vit=vCovKFs.begin(), vend=vCovKFs.end(); vit!=vend; vit++) {
                    Eigen::Vector3d Ow2 = (*vit)->GetTranslation();
                    glVertex3f(Ow(0),Ow(1),Ow(2));
                    glVertex3f(Ow2(0),Ow2(1),Ow2(2));
                }
            }
        }

        glEnd();
    }
}

}  // namespace SLAM_VIEWER
