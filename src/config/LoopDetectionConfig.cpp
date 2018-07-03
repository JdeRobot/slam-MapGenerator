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

#include "LoopDetectionConfig.h"

namespace MapGen {

    LoopDetectionConfig::LoopDetectionConfig(std::string filename) {
        cv::FileStorage fs(filename, cv::FileStorage::READ);
        if (!fs.isOpened()){
            BOOST_LOG_TRIVIAL(fatal) << "Fail to read the config file: " << filename;
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
            BOOST_LOG_TRIVIAL(fatal) << err_msg;
            throw std::runtime_error(err_msg);
        }

        fs["trajectory"] >> trajectory_;
        fs["Vocabulary"] >> vocabulary_;
        fs["loop_detection_threshold"] >> threshold_;

        // print nice info
        BOOST_LOG_TRIVIAL(info) << "==================================Config================================";
        BOOST_LOG_TRIVIAL(info) << "img_dir: " << img_dir_;
        BOOST_LOG_TRIVIAL(info) << "trajectory: " << trajectory_;
        BOOST_LOG_TRIVIAL(info) << "Vocabulary: " << vocabulary_;
        BOOST_LOG_TRIVIAL(info) << "loop_detection_threshold: " << threshold_;
        BOOST_LOG_TRIVIAL(info) << "========================================================================";
    }

    std::string LoopDetectionConfig::get_img_dir() {return img_dir_;}
    std::string LoopDetectionConfig::get_trajectory() {return trajectory_;}
    std::string LoopDetectionConfig::get_vocabulary() {return vocabulary_;}
    double LoopDetectionConfig::get_threshold() {return threshold_;}

}