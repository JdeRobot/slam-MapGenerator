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

#include "NodeConfig.h"

namespace MapGen {

    NodeConfig::NodeConfig(std::string filename) {
        cv::FileStorage fs(filename, cv::FileStorage::READ);
        if (!fs.isOpened()){
            LOG_ERROR << "Fail to read the config file: " << filename;
            throw std::runtime_error("Fail to read the config file: " + filename);
        }

        // fix img_path if the user forget to add tailing '/'
        fs["img_dir"] >> img_dir_;
        if ((img_dir_.length() > 0) && (img_dir_[img_dir_.length() - 1] != '/')){
            img_dir_.push_back('/');
        }
        // check directory exist
        struct stat sb;
        if ((stat(img_dir_.c_str(), &sb) == -1) || (!S_ISDIR(sb.st_mode))){
            std::string err_msg = "img_dir: " + img_dir_ + " does not exist.";
            LOG_ERROR << err_msg;
            throw std::runtime_error(err_msg);
        }

        fs["trajectory"] >> trajectory_;
        fs["Vocabulary"] >> vocabulary_;
        fs["loop_detection_threshold"] >> threshold_;

        // print nice info
        LOG_INFO << "==================================Config================================";
        LOG_INFO << "img_dir: " << img_dir_;
        LOG_INFO << "trajectory: " << trajectory_;
        LOG_INFO << "Vocabulary: " << vocabulary_;
        LOG_INFO << "loop_detection_threshold: " << threshold_;
        LOG_INFO << "========================================================================";
    }

    std::string NodeConfig::get_img_dir() {return img_dir_;}
    std::string NodeConfig::get_trajectory() {return trajectory_;}
    std::string NodeConfig::get_vocabulary() {return vocabulary_;}
    double NodeConfig::get_threshold() {return threshold_;}

}