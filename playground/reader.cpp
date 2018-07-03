#include <pcl/io/vtk_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

using namespace pcl::io;

int main(int argc, const char * argv[]){
    if (argc != 2){
        std::cerr << "[ERROR] Invalid usage." << std::endl;
        return 1;
    }

    pcl::PolygonMesh triangles;
    loadPolygonFileVTK(argv[1],triangles);
    std::cout << "[INFO] mesh loaded." << std::endl;

    pcl::PointCloud<pcl::PointXYZ> vertice;
    pcl::fromPCLPointCloud2(triangles.cloud, vertice);

    for (int i = 0; i < triangles.polygons.size(); i++){
        // get the first index
        int point_index = triangles.polygons[i].vertices[0];
        pcl::PointXYZ p = vertice[point_index];
    }

    return 0;
}