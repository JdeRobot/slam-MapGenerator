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



    ParamNamespace::ParamNamespace(const std::string& name_space) : name_space_(name_space),
                                                                   string_params_(),
                                                                   double_params_(),
                                                                   string_params_names_(),
                                                                   double_params_names_() {

    }

    ParamNamespace::ParamNamespace() {}

    void ParamNamespace::add_param(const std::string &name, const std::string &type) {
        if (type == "string")           string_params_names_.push_back(name);
        else if (type == "double")      double_params_names_.push_back(name);
        else{
            throw std::runtime_error("Parameter type not supported, not implemented yet!");
        }
    }

    void ParamNamespace::set_string_param(const std::string &name, const std::string &value) {
        string_params_[name] = value;
    }

    void ParamNamespace::set_double_param(const std::string &name, double value) {
        double_params_[name] = value;
    }

    std::string ParamNamespace::get_string_param(const std::string& name) {
        if (string_params_.count(name) == 0){
            LOG_ERROR << "Parameter Name: " << name << "not found in NameSpace " << name_space_ << std::endl;
            throw std::runtime_error("");
        }

        return string_params_[name];
    }

    double ParamNamespace::get_double_param(const std::string &name) {
        if (double_params_.count(name) == 0){
            LOG_ERROR << "Parameter Name: " << name << "not found in NameSpace " << name_space_ << std::endl;
            throw std::runtime_error("");
        }

        return double_params_[name];
    }

    void ParamNamespace::dump_to_fs(cv::FileStorage &fs) {
        if ((string_params_names_.size() + double_params_names_.size()) == 0){
            std::string comment = name_space_ + " has no local parameters.";
            fs.writeComment(comment);
            return;
        }

        fs << name_space_.c_str();

        fs << "{";

        for (const std::string& name : string_params_names_){
            fs << name.c_str() << string_params_[name].c_str();
        }
        for (const std::string& name : double_params_names_){
            fs << name.c_str() << double_params_[name];
        }

        fs << "}";
    }

    void ParamNamespace::read_from_fs(cv::FileNode &node) {
        for (const std::string& name : string_params_names_){
            node[name] >> string_params_[name];
        }

        for (const std::string& name : double_params_names_){
            node[name] >> double_params_[name];
        }

    }


    void ParamNamespace::print_all_params() {
        LOG_INFO << "Params Namespace: " << name_space_ << std::endl;
        for (const std::string& name : string_params_names_){
            LOG_INFO << "    " << name << " : " << string_params_[name] << std::endl;
        }
        for (const std::string& name : double_params_names_){
            LOG_INFO << "    " << name << " : " << double_params_[name] << std::endl;
        }
    }

    std::string ParamNamespace::get_ns_name() {
        return name_space_;
    }


    NodeConfig::NodeConfig() : namespace_map() {}

    void NodeConfig::set_string_param(const std::string &name_space, const std::string &param_name, const std::string& value) {
        namespace_map[name_space].set_string_param(param_name, value);
    }

    void NodeConfig::set_double_param(const std::string &name_space, const std::string &param_name,
                                      double value) {
        namespace_map[name_space].set_double_param(param_name,value);
    }


    std::string NodeConfig::get_string_param(const std::string &name_space, const std::string &name) {
        if (namespace_map.count(name_space) == 0){
            LOG_ERROR << "Namespace: " << name_space << "not found in NodeConfig" << std::endl;
            throw std::runtime_error("");
        }

        return namespace_map[name_space].get_string_param(name);
    }

    double NodeConfig::get_double_param(const std::string &name_space, const std::string &name) {
        if (namespace_map.count(name_space) == 0){
            LOG_ERROR << "Namespace: " << name_space << "not found in NodeConfig" << std::endl;
            throw std::runtime_error("");
        }

        return namespace_map[name_space].get_double_param(name);
    }

    void NodeConfig::add_namespace(const std::string &name_space) {
        namespace_map[name_space] = ParamNamespace(name_space);
    }

    void NodeConfig::add_param(const std::string &name_space, const std::string &name, const std::string &type) {
        namespace_map[name_space].add_param(name, type);
    }

    void NodeConfig::dump(const std::string &filename) {
        cv::FileStorage fs(filename.c_str(), cv::FileStorage::WRITE);
        for (auto it = namespace_map.begin(); it != namespace_map.end(); it ++){
            it->second.dump_to_fs(fs);
        }
        fs.release();
    }


    // TODO: delete this, this will not work
    NodeConfig::NodeConfig(std::string filename) : namespace_map() {
        cv::FileStorage fs(filename, cv::FileStorage::READ);

        if (!fs.isOpened()){
            LOG_ERROR << "Fail to read the config file: " << filename << std::endl;
            throw std::runtime_error("Fail to read the config file: " + filename);
        }
//
//        // fix img_path if the user forget to add tailing '/'
//        fs["img_dir"] >> img_dir_;
//        if ((img_dir_.length() > 0) && (img_dir_[img_dir_.length() - 1] != '/')){
//            img_dir_.push_back('/');
//        }
//        if (!is_dir_exist(img_dir_)){
//            img_dir_ = "";
//        }
//
//        fs["trajectory"] >> trajectory_;
//        fs["pointcloud"] >> pc_filename_;
//        fs["Vocabulary"] >> vocabulary_;
//        fs["use_trajectory"] >> use_trajectory_;
//        fs["loop_detection_threshold"] >> threshold_;
//
//        fs["use_fast_triangulation_recon"] >> use_fast_triangulation_recon_;
//        fs["use_poisson_recon"] >> use_poisson_recon_;
//        if (((int)use_fast_triangulation_recon_ + (int)use_poisson_recon_) > 1){
//            LOG_ERROR << "Enabled more than one surface reconstruction method." << std::endl;
//        }

        for (auto ns : namespace_map){
            cv::FileNode fs_node = fs[ns.first];
            ns.second.read_from_fs(fs_node);
        }

        // print nice info
        LOG_INFO << "==================================Config================================" << std::endl;
//        LOG_INFO << "img_dir: " << img_dir_ << std::endl;
//        LOG_INFO << "trajectory: " << trajectory_ << std::endl;
//        LOG_INFO << "Vocabulary: " << vocabulary_ << std::endl;
//        LOG_INFO << "loop_detection_threshold: " << threshold_ << std::endl;
        for (auto ns : namespace_map){
            ns.second.print_all_params();
        }
        LOG_INFO << "========================================================================" <<std::endl;
    }

    void NodeConfig::read_from_file(const std::string &filename) {
        cv::FileStorage fs(filename, cv::FileStorage::READ);

        // if not opened, exit now
        if (!fs.isOpened()){
            LOG_ERROR << "Fail to read the config file: " << filename << std::endl;
            throw std::runtime_error("Fail to read the config file: " + filename);
        }

        // read files
        for (auto it = namespace_map.begin(); it != namespace_map.end(); it++){
            cv::FileNode fs_node = fs[it->first];
//            std::string tmp;
//            fs_node["img_dir"] >> tmp;
            it->second.read_from_fs(fs_node);
        }

        // print nice info
        LOG_INFO << "==================================Config================================" << std::endl;
        for (auto it = namespace_map.begin(); it != namespace_map.end(); it++){
            cv::FileNode fs_node = fs[it->first];
//            std::string tmp;
//            fs_node["img_dir"] >> tmp;
            it->second.print_all_params();
        }
        LOG_INFO << "========================================================================" <<std::endl;
    }
}