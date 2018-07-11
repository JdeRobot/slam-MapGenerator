#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "logging_util.h"
#include "Map.h"
#include "Config.h"
#include "NodeConfig.h"
#include "surface_recon_util.h"

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
    MapGen::Camera camera;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    if (config.use_trajectory()) {
        // read in the trajectory file
        if (!MapGen::Config::ReadParameters(config.get_trajectory(), map, camera)) {
            LOG_ERROR << "fail to read the trajectory file at: " << config.get_trajectory() << std::endl;
            return 1;
        }

        // convert all points to PCL format
        auto points = map.GetAllMapPoints();
        for (auto point : points) {
            auto pos = point->GetWorldPos();
            cloud->push_back(PointXYZ(pos[0], pos[1], pos[2]));
        }
    }
    else{
        pcl::io::loadPCDFile(config.get_pointcloud(),*cloud);
    }

    // save all map points to PCL
    // pcl::io::savePCDFileASCII("cloud.xyz", *cloud);
    // LOG_DEBUG << "raw pointcloud saved to : cloud.xyz" << std::endl;


    // start recon
    pcl::PolygonMesh triangles;
    if (config.use_fast_triangulation_recon()){
        triangles = pcl_fast_surface_recon(cloud);
    }
    else if (config.use_poisson_recon()){
        triangles = pcl_poisson_recon(cloud);
    }
    pcl::io::saveVTKFile("mesh.vtk",triangles);
    LOG_DEBUG << "mesh saved to : mesh.vtk" << std::endl;
    return 0;
}