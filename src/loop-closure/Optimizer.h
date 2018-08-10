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

#ifndef SLAM_MAPGEN_OPTIMIZER_H
#define SLAM_MAPGEN_OPTIMIZER_H

#include "Map.h"
#include "LoopConnection.h"
#include "Connection.h"
#include "logging_util.h"

namespace MapGen {

    class Optimizer {
    public:
        Optimizer(std::vector<KeyFrame *> frames, const Camera& cam);

        // the nodes should be added sequentially
        void add_loop_closing(KeyFrame * frame_a, KeyFrame * frame_b);

        void run_optimizer();

    private:
        std::vector<Connection> seq_connections_;
        std::vector<LoopConnection> loop_connections_;

        Camera cam_;

        // the scaling factor used to rescale the relative pose (Sim3) solved by OpenCV
        // so that it could meet with the ground truth
        // should be initialized with several observation before loop closing
        // double scaling_factor_;
    };
}

#endif //SLAM_MAPGEN_OPTIMIZER_H
