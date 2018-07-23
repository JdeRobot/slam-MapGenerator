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
#include <map>

namespace MapGen {
    // =========================================================================
    //  Some helper functions
    // =========================================================================
    bool is_dir_exist(const std::string& dir_name);
    bool is_file_exist(const std::string& file_name);



    // =========================================================================
    //  ParamNamespace Class
    // =========================================================================
    class ParamNamespace{
    public:
        ParamNamespace();
        ParamNamespace(const std::string& name_space);

        // The type could be:
        //      string
        //      double
        void add_param(const std::string& name, const std::string& type);

        // TODO: check if the param_name is in the string params
        void set_string_param(const std::string& name, const std::string& value);
        void set_double_param(const std::string& name, double value);

        // TODO: what if it cause an error
        std::string get_string_param(const std::string& name);
        double get_double_param(const std::string& name);

        void dump_to_fs(cv::FileStorage& fs);
        void read_from_fs(cv::FileNode& node);

    private:
        std::string name_space_;

        std::map<std::string, std::string>  string_params_;
        std::map<std::string, double>  double_params_;

        std::vector<std::string>  string_params_names_;
        std::vector<std::string> double_params_names_;
    };


    // =========================================================================
    //  NodeConfig Class
    // =========================================================================
    class NodeConfig {
    public:
        NodeConfig();
        NodeConfig(std::string filename);

        void add_namespace(const std::string& name_space);
        void add_param(const std::string& name_space, const std::string& name, const std::string &type);

        void set_string_param(const std::string& name_space, const std::string& param_name, const std::string& value);
        void set_double_param(const std::string& name_space, const std::string& param_name, double value);

        std::string get_string_param(const std::string& name_space, const std::string& name);
        double get_double_param(const std::string& name_space, const std::string& name);

        void dump(const std::string& filename );


        std::string get_img_dir();

        std::string get_trajectory();
        std::string get_pointcloud();
        std::string get_vocabulary();
        bool use_trajectory();

        double get_threshold();

        bool use_fast_triangulation_recon();
        bool use_poisson_recon();

    private:
        std::map<std::string, ParamNamespace> namespace_map;

        std::string img_dir_;
        std::string trajectory_;
        std::string vocabulary_;
        std::string pc_filename_;
        double threshold_;

        bool use_trajectory_;
        bool use_fast_triangulation_recon_;
        bool use_poisson_recon_;
    };
}


#endif //SLAM_MAPGEN_LOOPDETECTIONCONFIG_H
