/**
 *
 *  Copyright (C) 2018 Jianxiong Cai <caijx AT shanghaitech.edu.cn>
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
