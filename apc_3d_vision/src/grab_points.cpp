/*
Copyright 2016 Australian Centre for Robotic Vision
*/

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <eigen_conversions/eigen_msg.h>

#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

#include <shape_msgs/Mesh.h>

#include <std_msgs/String.h>
#include <std_msgs/Bool.h>

#include <cv_bridge/cv_bridge.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/filter.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <string>
#include <vector>
#include <iostream>

#include "geometric_shapes/shapes.h"
#include "geometric_shapes/mesh_operations.h"
#include "geometric_shapes/shape_operations.h"

static const double PUBLISH_UPDATE_FREQUENCY = 10;  // Publish @ 10 Hz;

boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud;

void pointsCallback(const sensor_msgs::PointCloud2::ConstPtr msg) {
    pcl::fromROSMsg(*msg, *cloud);
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "grab_points_ros_node");
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_("~");
    cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);

    std::string depth_image_fn;
    std::string points_topic;
    int image_width;
    int image_height;

    nh_private_.getParam("depth_image_fn", depth_image_fn);
    nh_private_.getParam("points_topic", points_topic);
    nh_private_.getParam("image_width", image_width);
    nh_private_.getParam("image_height", image_height);

    printf("Topic = %s", points_topic.c_str());

    ros::Subscriber subPoints = nh_.subscribe(points_topic, 10, pointsCallback);

    ros::Rate publish_rate(PUBLISH_UPDATE_FREQUENCY);
    std::stringstream ss;

    while (nh_.ok()) {
        printf("Waiting for %s to come online...", points_topic.c_str());

        if (cloud->size() > 0) {
            printf("Received points\n");

            std::vector<int> indices;
            pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);

            if (cloud->size() > 0) {
                // pcl::PointCloud<pcl::PointXYZ>::iterator itr = (*cloud).begin();
                // for (; itr != (*cloud).end(); ++itr) {
                //     // ss << itr->x << ",";
                //     // ss << itr->y << ",";
                //     ss << itr->z << ",\n";
                // }

                for (int col = 0; col < image_width; col++) {
                    for (int row = 0; row < image_height; row++) {
                        ss << cloud->points[row*image_width + col].z << ",";
                    }
                    ss << "\n";
                }

                std::ofstream ofstr;
                ofstr.open(depth_image_fn.c_str());
                ofstr << ss.str().c_str();
                ofstr.close();

                printf("Saved points to file: %s\n", depth_image_fn.c_str());
            } else {
                printf("Cloud has no points! Exciting...\n");
            }

            ros::shutdown();
        }

        ros::spinOnce();
        publish_rate.sleep();
    }

    return 0;
}
