//
// Created by ernest on 18-8-9.
//

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

#include <stdio.h>
#include <string.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/imgproc/imgproc.hpp>

#include <pangolin/pangolin.h>
#include <thread>

#include "logging_util.h"

//#include <GLFW/glfw3.h>
//#include <GLUT/glut.h>

using namespace cv;
using namespace std;

void run_viewer();

int main(int argc, const char * argv[]){
    std::thread viewer(run_viewer);

    usleep(1000);

    std::string img_path = "/home/ernest/SLAM/datasets/sequence_43/undistorted_images/00307.jpg";

    Mat image;
    image = cv::imread(img_path, CV_LOAD_IMAGE_COLOR);   // Read the file

    if(! image.data )                              // Check for invalid input
    {
        cout <<  "Could not open or find the image" << std::endl ;
        return -1;
    }

//    namedWindow( "Display window", WINDOW_AUTOSIZE );// Create a window for display.
//    imshow( "Display window", image );                   // Show our image inside it.
//
//    waitKey(0);                                          // Wait for a keystroke in the window

    GLuint texture;
    glGenTextures(1, &texture);
    glBindTexture(GL_TEXTURE_2D, texture);
    glTexParameterf(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR);
    glTexParameterf(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);
    glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_WRAP_S , GL_REPEAT );
    glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT );
    glTexImage2D(GL_TEXTURE_2D, 0, 3, image.cols, image.rows, 0, GL_RGB, GL_UNSIGNED_BYTE, image.ptr());

    glBindTexture(GL_TEXTURE_2D, 1);
    glBegin(GL_QUADS);
    glTexCoord2f(0.0, 0.0); glVertex3f(-1.0, -1.0,  1.0);
    glTexCoord2f(1.0, 0.0); glVertex3f( 1.0, -1.0,  1.0);
    glTexCoord2f(1.0, 1.0); glVertex3f( 1.0,  1.0,  1.0);
    glTexCoord2f(0.0, 1.0); glVertex3f(-1.0,  1.0,  1.0);
    glEnd();


    viewer.join();

    return 0;
}


void run_viewer(){
    int w, h, mw;

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
    pangolin::Var<bool> menuReset("menu.Reset", false, false);

    // Define Camera Render Object (for view / scene browsing)
    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(w, h, 1, 1, w/2, h/2, 0.1, 1000),
            pangolin::ModelViewLookAt(100, 100, 100, 0, 0, 0, 0.0,-1.0, 0.0));

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

    }


    LOG_INFO << "UI thread finished, exiting..." << std::endl;
}