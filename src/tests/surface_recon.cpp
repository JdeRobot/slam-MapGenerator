#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "logging_util.h"
#include "Map.h"
#include "Config.h"
#include "NodeConfig.h"

using namespace pcl;
using namespace MapGen;

int main(int argc, const char *argv[]) {
    init_logging();

    if (argc != 2) {
        LOG_ERROR << "Usage error" << std::endl;
        return 1;
    }

    NodeConfig config(argv[1]);
    MapGen::Map map;

    // read in the trajectory file
    if (!MapGen::Config::ReadParameters(config.get_trajectory(), map)) {
        LOG_ERROR << "fail to read the trajectory file at: " << config.get_trajectory() << std::endl;
        return 1;
    }

    // convert all points to PCL format
    PointCloud<PointXYZ> cloud;
    auto points = map.GetAllMapPoints();
    for (auto point : points){
        auto pos = point->GetWorldPos();
        cloud.push_back(PointXYZ(pos[0], pos[1], pos[2]));
    }

    // save all map points to PCL
    pcl::io::savePCDFileASCII("cloud.xyz", cloud);
    LOG_DEBUG << "cloud saved" << std::endl;
    return 0;
}