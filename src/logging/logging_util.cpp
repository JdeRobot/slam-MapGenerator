#include "logging_util.h"

void init_logging(){
    if (CMAKE_BUILD_TYPE == "Debug") {
        std::cout << "set logger to Debug" << std::endl;
        boost::log::core::get()->set_filter(
                boost::log::trivial::severity >= boost::log::trivial::debug
        );
    }
    else{
        std::cout << "set logger to Release" << std::endl;
        boost::log::core::get()->set_filter(
                boost::log::trivial::severity >= boost::log::trivial::info
        );
    }
}


inline void LOG_INFO(const std::string& msg){
    BOOST_LOG_TRIVIAL(info) << msg;
}