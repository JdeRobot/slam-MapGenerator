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
    pangolin::Var<bool> menuRefresh("menu.Refresh", false, true);
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
