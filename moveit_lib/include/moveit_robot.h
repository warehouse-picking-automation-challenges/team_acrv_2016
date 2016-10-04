#ifndef MOVEIT_ROBOT_H
#define MOVEIT_ROBOT_H

#include <eigen_conversions/eigen_msg.h>
#include <ros/ros.h>
#include <ros/time.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <string>

// use this define for enabling debug information with move_group planner and
// execution
// #define DEBUG

class moveit_robot {
   public:
    // Class Constructor
    moveit_robot(std::string robotGroup);

    // Initialise move_group given planner as string and velocity scaling factor
    // from 0 to 1
    void initRobot(std::string planner = "RRTStarkConfigDefault",
                   float velocity = 0.75);

    // move to a goal pose and frame id, using the number of max_attempts
    // The planning time will start at max_time/max_attempts and increase
    // to max time for the last attempt
    // velocity scale will determine the speed of the motion
    bool moveTo(Eigen::Affine3d pose_eigen, std::string frame_id,
                int max_attempts, double timeout, double max_planning_time,
                float velocity_scale = 1.0);
    bool moveTo(geometry_msgs::PoseStamped pose, int max_attempts,
                double timeout, double max_planning_time,
                float velocity_scale = 1.0);
    bool moveTo(std::string frame_id, Eigen::Vector3d position,
                Eigen::Vector4d orientation, int max_attempts, double timeout,
                double max_planning_time, float velocity_scale = 1.0);
    double moveToCartesianPath(std::vector<geometry_msgs::Pose> waypoints,
                               int max_attempts, double timeout,
                               double max_planning_time, float velocity_scale,
                               float step_size, float jump_threshold, double min_path_completion);
    bool moveToNamed(std::string namedGoal, int max_attempts, double timeout,
                     double max_planning_time, float velocity_scale = 1.0);
    bool moveToOrientationConstraint(geometry_msgs::PoseStamped pose,
                                     geometry_msgs::Quaternion qConstraint,
                                     std::string link_name, int max_attempts,
                                     double timeout, double max_planning_time,
                                     float velocity_scale = 1.0);

    // execute a planned move!
    bool executePlan();

    void configureWorkspace();

    moveit::planning_interface::MoveGroup::Plan robot_plan_;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit::planning_interface::MoveGroup move_group;

    float velocity_scale_;

    std::string ee_link_name;
    std::string planning_frame;
};

#endif  // MOVEIT_ROBOT_H
