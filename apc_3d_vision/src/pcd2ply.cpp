/*
Copyright 2016 Australian Centre for Robotic Vision
*/

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/filter.h>
#include <boost/shared_ptr.hpp>

#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>


int main(int argc, char** argv) {
    std::string input_cloud_fn = (std::string)argv[1];
    std::string output_cloud_fn = (std::string)argv[2];

    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> input_cloud;
    input_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile<pcl::PointXYZ>(input_cloud_fn, *input_cloud);

    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*input_cloud, *input_cloud, indices);

    // Save data to pcd file:
    pcl::io::savePLYFileASCII(output_cloud_fn, *input_cloud);
    std::cout << "Saved ply" << std::endl;

    return EXIT_SUCCESS;
}
