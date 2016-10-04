#ifndef ITEM_H
#define ITEM_H

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <geometric_shapes/shape_operations.h>

#include <moveit/move_group_interface/move_group.h>

#include <baxter_core_msgs/EndEffectorCommand.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <math.h>

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_lib/move_robot_named.h>

#include <ros/ros.h>

#include <iostream>

#include <string>
#include <vector>


class Item {
 public:
    Item();

    Item(const float positionX, const float positionY, const float positionZ,
        const float orientationX, const float orientationY,
        const float orientationZ, std::string item_id,
        moveit_msgs::PlanningScene* planning_scene_ptr);

    Item(const float positionX, const float positionY, const float positionZ,
        const float orientationX, const float orientationY,
        const float orientationZ, std::string item_id);

    Item(geometry_msgs::Pose pose, std::string item_id,
        moveit_msgs::PlanningScene* planning_scene_ptr);

    Item(geometry_msgs::Pose pose, std::string item_id);

    void addCollisionItem();

    float getPosX() const;

    float getPosY() const;

    float getPosZ() const;

    float getOrienX() const;

    float getOrienY() const;

    float getOrienZ() const;

    void setPose(const float posX, const float posY, const float posZ,
        const float orienX, const float orienY, const float orienZ);

    void setPose(geometry_msgs::Pose pose);

    void broadcastTf();

    ~Item();

 protected:
    moveit_msgs::PlanningScene* _planning_scene_ptr;
    std::string _item_id;
    geometry_msgs::Pose _pose; // Pose of the item
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    moveit::planning_interface::PlanningSceneInterface
    planning_scene_interface;
    tf::Transform _tfItem;
    tf::Transform _tfItemGoal;
    tf::TransformBroadcaster br;  // Transform broadcaster

    float _positionX, _positionY, _positionZ, _orientationX, _orientationY,
    _orientationZ, _orientationW;
};

#endif
