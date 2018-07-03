//
// Created by ernest on 18-7-2.
//

#include <iostream>
#include <string>
#include <thread>
#include "Map.h"
#include "Config.h"
#include "Viewer.h"
#include "MapDrawer.h"


using namespace std;
using namespace MapGen;

int main(int argc, const char * argv[]){
    vector<string> vFilenames;
    MapGen::Map map;

    // Create user interface
    MapGen::MapDrawer * mdrawer = new MapGen::MapDrawer(&map);

    MapGen::Viewer* viewer = nullptr;
    std::thread* tviewer = nullptr;

    viewer = new MapGen::Viewer(mdrawer);
    tviewer = new std::thread(&MapGen::Viewer::Run, viewer);

    // Wait until threads start
    usleep(1000*1e3);

    // Main loop
    while (!viewer->isFinished()) {
        // Wait
        usleep(30*1e3);
    }

    viewer->RequestFinish();
    while (!viewer->isFinished())
        usleep(5000);

    tviewer->join();

    return 0;
}