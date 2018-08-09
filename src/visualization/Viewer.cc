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

#include "Viewer.h"
#include <pangolin/pangolin.h>
#include <unistd.h>
#include "Config.h"

using std::mutex;
using std::unique_lock;

namespace MapGen {

Viewer::Viewer(MapDrawer *pMapDrawer):
    mpMapDrawer(pMapDrawer), mbFinishRequested(false), mbFinished(true) {
}

void Viewer::Run() {
    int w, h, mw;

    mbFinished = false;
    w = 1024;
    h = 768;
    mw = 175;

    pangolin::CreateWindowAndBind("SLAM Viewer", w, h);

    // 3D Mouse handler requires depth testing to be enabled
    glEnable(GL_DEPTH_TEST);

    // Issue specific OpenGl we might need
    glEnable (GL_BLEND);
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::CreatePanel("menu").SetBounds(0.0, 1.0, 0.0,pangolin::Attach::Pix(mw));
    pangolin::Var<bool> menuShowPoints("menu.Show Points", true, true);
    pangolin::Var<bool> menuShowKeyFrames("menu.Show KeyFrames", true, true);
    pangolin::Var<bool> menuShowGraph("menu.Show Graph", true, true);
    pangolin::Var<bool> menuShowSurface("menu.Show Surface", true, true);
    pangolin::Var<bool> menuShowImg("menu.Show Image", false, true);
    pangolin::Var<bool> menuReset("menu.Reset", false, false);

    // Define Camera Render Object (for view / scene browsing)
    pangolin::OpenGlRenderState s_cam(
        pangolin::ProjectionMatrix(w, h, Config::ViewpointF(), Config::ViewpointF(), w/2, h/2, 0.1, 1000),
        pangolin::ModelViewLookAt(Config::ViewpointX(), Config::ViewpointY(), Config::ViewpointZ(), 0, 0, 0, 0.0,-1.0, 0.0));

    // Add named OpenGL viewport to window and provide 3D Handler
    pangolin::View& d_cam = pangolin::CreateDisplay()
        .SetBounds(0.0, 1.0, pangolin::Attach::Pix(mw), 1.0, -w/(float)h)
        .SetHandler(new pangolin::Handler3D(s_cam));

    pangolin::OpenGlMatrix Twc;
    Twc.SetIdentity();

    while (!pangolin::ShouldQuit()) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
        if (menuShowKeyFrames || menuShowGraph)
            mpMapDrawer->DrawKeyFrames(menuShowKeyFrames, menuShowGraph);
        if (menuShowPoints)
            mpMapDrawer->DrawMapPoints();

        if (menuShowSurface){
            mpMapDrawer->DrawSurface();
        }

        if (menuShowImg) {
            // load the image (TODO: for testing only)
            std::string img_path_1 = "/home/ernest/SLAM/datasets/mydata/images/10044.png";
            std::string img_path_2 = "/home/ernest/SLAM/datasets/mydata/images/10001.png";

            texture[0] = SOIL_load_OGL_texture // load an image file directly as a new OpenGL texture
                    (
                            img_path_1.c_str(),
                            SOIL_LOAD_AUTO,
                            SOIL_CREATE_NEW_ID,
                            SOIL_FLAG_MIPMAPS | SOIL_FLAG_INVERT_Y | SOIL_FLAG_NTSC_SAFE_RGB | SOIL_FLAG_COMPRESS_TO_DXT
                    );
            texture[1] = SOIL_load_OGL_texture // load an image file directly as a new OpenGL texture
                    (
                            img_path_2.c_str(),
                            SOIL_LOAD_AUTO,
                            SOIL_CREATE_NEW_ID,
                            SOIL_FLAG_MIPMAPS | SOIL_FLAG_INVERT_Y | SOIL_FLAG_NTSC_SAFE_RGB | SOIL_FLAG_COMPRESS_TO_DXT
                    );


// allocate a texture name

            glEnable(GL_TEXTURE_2D);
//            glEnable(GL_BLEND);
//            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
            glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
            glBindTexture(GL_TEXTURE_2D, texture[0]);
//            glEnable (GL_TEXTURE_2D);
            glBegin(GL_QUADS);
//            glPixelStorei (GL_UNPACK_ALIGNMENT, 1);
//            glColor3d(1,1,0);
            glTexCoord2f(0.0f, 0.0f); glVertex3f(0.0f, 0.0f,  0.0f);
            glTexCoord2f(1.0f, 0.0f); glVertex3f( 1.0f, 0.0f,  0.0f);
            glTexCoord2f(1.0f, 1.0f); glVertex3f( 1.0f,  1.0f,  0.0f);
            glTexCoord2f(0.0f, 1.0f); glVertex3f(0.0f,  1.0f,  0.0f);
            glEnd();
            glDisable(GL_TEXTURE_2D);
        }

//        if (menuShowImg){
//            std::string img_path = "/home/ernest/SLAM/datasets/mydata/images/10044.png";
//            pangolin::GlTexture texture;
//            pangolin::TypedImage img = pangolin::LoadImage(img_path.c_str(), pangolin::ImageFileTypePng);
//
//            texture.Reinitialise(img.w, img.h, GL_RGB, false, 0, GL_RGB, GL_UNSIGNED_BYTE);
//            texture.Upload(img.ptr , GL_RGB , GL_UNSIGNED_BYTE);
//            texture.RenderToViewport();
//        }


//        // TODO: for test only: showing a surface
//        if (menuShowSurfaceTest){
//            Eigen::Vector3f v1(0.0f,0.0f,-1.0f);
//            Eigen::Vector3f v2(1.0f,0.0f,-1.0f);
//            Eigen::Vector3f v3(0.0f,0.0f,0.0f);
//            Eigen::Vector3f v4(1.0f,0.0f,0.0f);
//            Eigen::Vector3f v5(0.0f,-1.0f,0.5f);
//            Eigen::Vector3f v6(1.0f,-1.0f,0.5f);
//            mpMapDrawer->DrawTriangle(v1,v2,v3,true);
//            mpMapDrawer->DrawTriangle(v2,v3,v4,true);
//            mpMapDrawer->DrawTriangle(v3,v4,v5,true);
//            mpMapDrawer->DrawTriangle(v4,v5,v6,true);
//            // std::cout << "[DEBUG, Viewer.cc] " << "=============================" << std::endl;
//        }

        pangolin::FinishFrame();


        if (menuReset) {
            menuShowGraph = true;
            menuShowKeyFrames = true;
            menuShowPoints = true;
            menuReset = false;
        }

        if (CheckFinish())
            break;
    }

    SetFinish();

	LOG_INFO << "UI thread finished, exiting..." << std::endl;
}

void Viewer::RequestFinish() {
    unique_lock<mutex> lock(mMutexFinish);
    mbFinishRequested = true;
}

bool Viewer::CheckFinish() {
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinishRequested;
}

void Viewer::SetFinish() {
    unique_lock<mutex> lock(mMutexFinish);
    mbFinished = true;
}

bool Viewer::isFinished() {
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinished;
}

}  // namespace SLAM_VIEWER
