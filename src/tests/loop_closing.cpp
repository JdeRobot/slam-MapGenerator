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

#include "Config.h"
#include "NodeConfig.h"
#include "Map.h"
#include "LoopDetector.h"
#include "LoopConnection.h"
#include "logging_util.h"
#include "Camera.h"
#include "Optimizer.h"

using namespace MapGen;

int main (int argc, const char * argv[]){
    init_logging();

    if (argc != 2){
        LOG_ERROR << "Usage error" <<std::endl;
        return 1;
    }


    NodeConfig config(argv[1]);
    Map map;
    Camera camera;


    // read in the trajectory file
    if(!MapGen::Config::ReadParameters(config.get_trajectory(), map, camera)){
        LOG_ERROR << "fail to read the trajectory file at: " << config.get_trajectory() << std::endl;
        return 1;
    }

    // detect loops
    MapGen::LoopDetector detector(map,config.get_img_dir(), config.get_vocabulary(), config.get_threshold());

    // print out the loop detection pair
    auto closing_pairs = detector.getLoopClosingPairs();
    for (auto p : closing_pairs){
        LOG_INFO << "detected loop closing pair: " << p.first->GetFilename() << " & "
                                << p.second->GetFilename() << std::endl;
    }

    LOG_INFO << "Detection Completed" << std::endl;


    // try to close the loop
    // create a BFMatcher
//    cv::BFMatcher matcher;
//    vector<LoopConnection *> loop_connections;
//    for (auto p : closing_pairs){
//        loop_connections.push_back(new LoopConnection(p, camera.get_intrinsic_matrix()));
//    }

    Optimizer opt(map.GetAllKeyFrames(),camera.get_intrinsic_matrix());


    // TODO: delete all elements in loop_connections (memory leak)

    return 0;
}