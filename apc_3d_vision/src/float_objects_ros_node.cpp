/*
Copyright 2016 Australian Centre for Robotic Vision
*/


#include <apc_3d_vision.hpp>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <tf/transform_listener.h>

#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

#include <cv_bridge/cv_bridge.h>
#include <pcl_ros/point_cloud.h>

#include <string>
#include <vector>
#include <iostream>


static const double PUBLISH_UPDATE_FREQUENCY = 0.0;

boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> kinect_cloud;
boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> registered_empty_shelf;

void registered_shelf_cloud_callback(
    const sensor_msgs::PointCloud2::ConstPtr msg) {

    pcl::fromROSMsg(*msg, *registered_empty_shelf);
}

void kinect_points_callback(const sensor_msgs::PointCloud2::ConstPtr msg) {
    pcl::fromROSMsg(*msg, *kinect_cloud);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "float_objects_ros_node");
    ros::NodeHandle nh;
    ros::NodeHandle local_nh("~");
    Apc3dVision apc_vis;
    kinect_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    registered_empty_shelf.reset(new pcl::PointCloud<pcl::PointXYZ>);

    double distanceThreshold;

    // get params local to empty_shelf_publisher node
    local_nh.param("distanceThreshold", distanceThreshold, 1.0);

    std::string topicRegisteredShelfPoints = "/registered_empty_shelf";
    std::string topicKinectPoints = "/kinect2/qhd/points";

    ros::Subscriber sub_registered_shelf_points = nh.subscribe(
        topicRegisteredShelfPoints, 10, registered_shelf_cloud_callback);
    ros::Subscriber sub_kinect_points = nh.subscribe(
        topicKinectPoints, 10, kinect_points_callback);

    ros::Publisher pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>(
        "floating_objects", 1);

    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> floating_objects;
    floating_objects.reset(new pcl::PointCloud<pcl::PointXYZ>);

    Apc3dVision::segment_differences_params_t params;

    params.input_cloud = kinect_cloud;
    params.target_cloud = registered_empty_shelf;
    params.output_cloud = floating_objects;
    params.distanceThreshold = distanceThreshold;
    params.verbose = true;

    bool isCloudSegmented = false;

    std::cout << "\nPublishing floating objects on /floating_objects.\n";
    sensor_msgs::PointCloud2::Ptr msg (new sensor_msgs::PointCloud2);

    while (nh.ok()) {
        ros::spinOnce();
        if (kinect_cloud->size() > 0 && registered_empty_shelf->size() > 0) {
            isCloudSegmented = apc_vis.segment_differences(params);

            if (isCloudSegmented) {
                std::cout << "\nObject segmentation success.\n";
            } else {
                std::cout
                    << "\nObject segmentation failure.\n\nExiting now...\n";
                return 0;
            }

            pcl::toROSMsg(*floating_objects, *msg);
            msg->header.frame_id = "kinect2_link";

            pub.publish(msg);
            ros::Duration(PUBLISH_UPDATE_FREQUENCY).sleep();
        }
    }

    return 0;
}
