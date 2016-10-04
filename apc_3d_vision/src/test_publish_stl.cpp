/*
Copyright 2016 Australian Centre for Robotic Vision
*/


#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

#include <shape_msgs/Mesh.h>

#include <Eigen/Core>

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

#include <cv_bridge/cv_bridge.h>
#include <pcl_ros/point_cloud.h>

#include <string>
#include <vector>
#include <iostream>

#include "geometric_shapes/shapes.h"
#include "geometric_shapes/mesh_operations.h"
#include "geometric_shapes/shape_operations.h"

static const double PUBLISH_UPDATE_FREQUENCY = 1.0;

int main(int argc, char** argv) {
    ros::init(argc, argv, "test_publish_stl");
    ros::NodeHandle nh;

    // ---------------- Shelf CAD Model ---------------------------------

    ros::Publisher collision_object_publisher =
        nh.advertise<moveit_msgs::CollisionObject>("/collision_object", 0);

    moveit_msgs::CollisionObject collision_object;

    collision_object.header.frame_id = "/kinect2_link";
    collision_object.header.stamp = ros::Time::now();
    collision_object.id = "shelf";
    shapes::Mesh* m = shapes::createMeshFromResource(
        "package://apc_3d_vision/models/pod_lowres.stl");

    shapes::ShapeMsg co_mesh_msg;
    shapes::constructMsgFromShape(m, co_mesh_msg);

    shape_msgs::Mesh co_mesh;
    co_mesh = boost::get<shape_msgs::Mesh>(co_mesh_msg);

    geometry_msgs::Pose shelf_pose;
    Eigen::Matrix3f shelf_rotation;
    shelf_rotation <<
          0.999291,  0.025606, 0.0275546,
        0.00483011, -0.813819,  0.581096,
          0.037304, -0.580552, -0.813366;
    Eigen::Quaternionf shelf_rotation_quaternion(shelf_rotation);
    shelf_pose.position.x = -0.10305;
    shelf_pose.position.y =  0.80056;
    shelf_pose.position.z =   2.1014;
    shelf_pose.orientation.x = shelf_rotation_quaternion.x();
    shelf_pose.orientation.y = shelf_rotation_quaternion.y();
    shelf_pose.orientation.z = shelf_rotation_quaternion.z();
    shelf_pose.orientation.w = shelf_rotation_quaternion.w();


    // shelf_rotation_quaternion = Eigen::Matrix3f(shelf_rotation);


   //   0.999291  0.025606 0.0275546 -0.103059
   // 0.00483011 -0.813819  0.581096  0.800566
   //   0.037304 -0.580552 -0.813366   2.10149
   //         0         0         0         1



    collision_object.meshes.push_back(co_mesh);
    collision_object.mesh_poses.push_back(shelf_pose);
    collision_object.operation = collision_object.ADD;


    // ------------------- BOX PRIMITIVE ----------------------------------

    // moveit_msgs::CollisionObject collision_object;
    // collision_object.header.frame_id = "/right_wrist";
    // collision_object.header.stamp = ros::Time::now();
    //
    // /* The id of the object is used to identify it. */
    // collision_object.id = "box1";
    //
    // /* Define a box to add to the world. */
    // shape_msgs::SolidPrimitive primitive;
    // primitive.type = primitive.BOX;
    // primitive.dimensions.resize(3);
    // primitive.dimensions[0] = 1.4;
    // primitive.dimensions[1] = 1.1;
    // primitive.dimensions[2] = 1.4;
    //
    // /* A pose for the box (specified relative to frame_id) */
    // geometry_msgs::Pose box_pose;
    // box_pose.orientation.w = 1.0;
    // box_pose.position.x =  0.6;
    // box_pose.position.y = -0.4;
    // box_pose.position.z =  1.2;
    //
    // collision_object.primitives.push_back(primitive);
    // collision_object.primitive_poses.push_back(box_pose);
    // collision_object.operation = collision_object.ADD;


    // ---------------- Adding the collision object into the planning scene
    // static const std::string ROBOT_DESCRIPTION = "robot_description";
    // planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_(
    //     new planning_scene_monitor::PlanningSceneMonitor(ROBOT_DESCRIPTION));
    // ros::spinOnce();
    // ros::Duration(1).sleep();
    // // static const std::string PLANNING_SCENE_TOPIC = "planning_scene";
    // static const std::string PLANNING_SCENE_TOPIC =
    //     "/move_group/monitored_planning_scene";
    // std::string planning_scene_topic_ = PLANNING_SCENE_TOPIC;
    //
    // if (planning_scene_monitor_->getPlanningScene()) {
    //     // Optional monitors to start:
    //     // planning_scene_monitor_->startWorldGeometryMonitor();
    //     // planning_scene_monitor_->startSceneMonitor(
    //     //     "/move_group/monitored_planning_scene");
    //     // planning_scene_monitor_->startStateMonitor(
    //     //     "/joint_states", "/attached_collision_object");
    //
    //     planning_scene_monitor_->startPublishingPlanningScene(
    //         planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE,
    //         planning_scene_topic_);
    //
    //     // planning_scene_monitor_->getPlanningScene()->setName(
    //     //     "visual_tools_scene");
    //     planning_scene_monitor::LockedPlanningSceneRW scene(
    //         planning_scene_monitor_);
    //
    //     // hack to prevent bad transforms
    //     scene->getCurrentStateNonConst().update();
    //     collision_object.header.stamp = ros::Time::now();
    //     scene->processCollisionObjectMsg(collision_object);
    //     ros::spinOnce();
    //     ros::Duration(1).sleep();
    //     planning_scene_monitor_->triggerSceneUpdateEvent(
    //         planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE);
    //     ros::spinOnce();
    //     std::cout << "Yoooooooo Good\n";
    //     // scene->setObjectColor(msg.id, getColor(color));
    // } else {
    //     std::cout << "Yooooooooo Fail\n";
    //     // ROS_ERROR_STREAM_NAMED(name_, "Planning scene not configured");
    // }

    // ----------------

    while (nh.ok()) {
        collision_object.header.stamp = ros::Time::now();
        collision_object_publisher.publish(collision_object);
        ros::spinOnce();
        ros::Duration(PUBLISH_UPDATE_FREQUENCY).sleep();
    }

    return 0;
}
