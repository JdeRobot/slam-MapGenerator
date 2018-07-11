//
// Created by ernest on 18-7-11.
//

#ifndef SLAM_MAPGEN_JDEROBOTIO_H
#define SLAM_MAPGEN_JDEROBOTIO_H

#include <fstream>
#include "Map.h"
#include "Camera.h"
#include "logging_util.h"

namespace MapGen {

    class JdeRobotIO {
    public:
        static void saveTrajectory(Map &map, const Camera& cam, const std::string &filename);
    };

}


#endif //SLAM_MAPGEN_JDEROBOTIO_H
