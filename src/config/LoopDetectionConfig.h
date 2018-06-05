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


#ifndef SLAM_MAPGEN_LOOPDETECTIONCONFIG_H
#define SLAM_MAPGEN_LOOPDETECTIONCONFIG_H

#include <string>
#include <opencv2/core/persistence.hpp>
#include <exception>
#include <boost/log/trivial.hpp>

namespace MapGen {
    class LoopDetectionConfig {
    public:
        explicit LoopDetectionConfig(std::string filename);

        std::string get_img_dir();

        std::string get_trajectory();

        std::string get_vocabulary();

        double get_threshold();

    private:
        std::string img_dir_;
        std::string trajectory_;
        std::string vocabulary_;
        double threshold_;
    };
}


#endif //SLAM_MAPGEN_LOOPDETECTIONCONFIG_H
