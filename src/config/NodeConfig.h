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
#include <sys/stat.h>
#include <logging_util.h>

namespace MapGen {

    bool is_dir_exist(const std::string& dir_name);
    bool is_file_exist(const std::string& file_name);

    class NodeConfig {
    public:
        explicit NodeConfig(std::string filename);

        std::string get_img_dir();

        std::string get_trajectory();
        std::string get_pointcloud();
        std::string get_vocabulary();
        bool use_trajectory();

        double get_threshold();

    private:
        std::string img_dir_;
        std::string trajectory_;
        std::string vocabulary_;
        std::string pc_filename_;
        double threshold_;

        bool use_trajectory_;
    };
}


#endif //SLAM_MAPGEN_LOOPDETECTIONCONFIG_H
