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
    bool is_dir_exist(const std::string& dir_name){
        struct stat sb;
        if ((stat(dir_name.c_str(), &sb) == -1) || (!S_ISDIR(sb.st_mode))){
            return false;
        }
        return true;
    }


    bool is_file_exist(const std::string& file_name){
        struct stat sb;
        if ((stat(file_name.c_str(), &sb) == -1) || (!S_ISREG(sb.st_mode))){
            return false;
        }
        return true;
    }

    NodeConfig::NodeConfig(std::string filename) {
        cv::FileStorage fs(filename, cv::FileStorage::READ);
        if (!fs.isOpened()){
            LOG_ERROR << "Fail to read the config file: " << filename << std::endl;
            throw std::runtime_error("Fail to read the config file: " + filename);
        }

        // fix img_path if the user forget to add tailing '/'
        fs["img_dir"] >> img_dir_;
        if ((img_dir_.length() > 0) && (img_dir_[img_dir_.length() - 1] != '/')){
            img_dir_.push_back('/');
        }
        if (!is_dir_exist(img_dir_)){
            img_dir_ = "";
        }

        fs["trajectory"] >> trajectory_;
        fs["pointcloud"] >> pc_filename_;
        fs["Vocabulary"] >> vocabulary_;
        fs["use_trajectory"] >> use_trajectory_;
        fs["loop_detection_threshold"] >> threshold_;

        fs["use_fast_triangulation_recon"] >> use_fast_triangulation_recon_;
        fs["use_poisson_recon"] >> use_poisson_recon_;
        if (((int)use_fast_triangulation_recon_ + (int)use_poisson_recon_) > 1){
            LOG_ERROR << "Enabled more than one surface reconstruction method." << std::endl;
        }

        // print nice info
        LOG_INFO << "==================================Config================================" << std::endl;
        LOG_INFO << "img_dir: " << img_dir_ << std::endl;
        LOG_INFO << "trajectory: " << trajectory_ << std::endl;
        LOG_INFO << "Vocabulary: " << vocabulary_ << std::endl;
        LOG_INFO << "loop_detection_threshold: " << threshold_ << std::endl;
        LOG_INFO << "========================================================================" <<std::endl;
    }

    std::string NodeConfig::get_img_dir() {
        if (img_dir_.size() == 0){
            LOG_ERROR << "img_dir: " << img_dir_ << " does not exist." << std::endl;
            throw std::runtime_error("");
        }

        return img_dir_;
    }
    std::string NodeConfig::get_trajectory() {return trajectory_;}
    std::string NodeConfig::get_pointcloud() {return pc_filename_;}
    std::string NodeConfig::get_vocabulary() {return vocabulary_;}

    double NodeConfig::get_threshold() {return threshold_;}

    bool NodeConfig::use_trajectory() {
        return use_trajectory_;
    }

    bool NodeConfig::use_fast_triangulation_recon(){return use_fast_triangulation_recon_;}
    bool NodeConfig::use_poisson_recon(){return use_poisson_recon_;}
}