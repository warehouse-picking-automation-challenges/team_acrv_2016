#ifndef SHELF_SCAN_H
#define SHELF_SCAN_H

#include <cv_bridge/cv_bridge.h>
#include <eigen_conversions/eigen_msg.h>
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <ros/time.h>
#include <std_msgs/Empty.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <string>
#include "moveit_lib/move_robot_named.h"
#include "moveit_lib/move_robot_pose.h"
#include "moveit_lib/move_robot_pose_array.h"
#include "moveit_lib/move_robot_tf.h"
#include <thread>         // std::this_thread::sleep_for
#include <chrono>         // std::chrono::seconds

// use this define for enabling debug information with move_group planner and
// execution
// #define DEBUG

class shelf_scanner {
   public:
    // Class Constructor
    shelf_scanner(std::string move_group);
    shelf_scanner(std::string move_group, std::string bin_letter);

    void init();
    void reinit(std::string move_group,
                geometry_msgs::PoseStamped initial_pose);
    void setShelfRadius(float scan_radius);
    bool createSimplePath(Eigen::Vector4d start_q_eigen,
                          Eigen::Vector3d delta_xyz,
                          Eigen::Vector3d scan_offset);
    void createConePath(float radius, float distance, int segments);
    void createConePath2(float radius, float distance, int segments);
    void generatePath();
    bool execute();

    ros::NodeHandle nh_;
    geometry_msgs::PoseStamped shelf_pose;
    std::string bin_name;
    ros::Publisher vis_pub;
    ros::Publisher vis_pub2;
    ros::Publisher reset_pub;
    ros::Publisher pause_pub;
    ros::Publisher unpause_pub;

    moveit_lib::move_robot_pose pose_srv;
    moveit_lib::move_robot_pose_array pose_array_srv;
    std_msgs::Empty empty_msg;

    ros::ServiceClient pose_client;
    ros::ServiceClient pose_array_client;
    Eigen::Vector4d start_q;
    Eigen::Vector3d delta_xyz, scan_offset;
    XmlRpc::XmlRpcValue shelf_layout_;
    tf::TransformListener tf_listener;
    geometry_msgs::PoseStamped orig_pose;
    ros::Time common_time;

    geometry_msgs::PoseArray waypoints;
    float radius;
    float distance;
    int segments;

    std::vector<double> OFFSET_FROM_SHELF = {-0.45, -0.35};
    std::vector<double> HEIGHT_ABOVE_SHELF = {0.0, -0.05};
    std::vector<double> ANGLE = {-1.275, -M_PI / 3};
};

#endif  // SHELF_SCAN_H
