#include "moveit_robot_node.h"
#include <string>
#include <vector>
#include "moveit_lib/move_robot_named.h"
#include "moveit_lib/move_robot_pose.h"
#include "moveit_lib/move_robot_pose_array.h"
#include "moveit_lib/move_robot_pose_constrained.h"
#include "moveit_lib/move_robot_tf.h"
#include "moveit_robot.h"

#define DEFAULT_PLANNING_TIME 5.0
#define MAX_ATTEMPTS 3
#define MOVE_TIMEOUT 3.0
#define WAIT_TIMEOUT 1.25

ros::Publisher pose_pub, pose_array_pub;
tf::TransformListener *tf_listener;

bool move_robot_tf(moveit_lib::move_robot_tf::Request &req,
                   moveit_lib::move_robot_tf::Response &res) {
    moveit_robot robot(req.move_group.data);

    tf::StampedTransform target_transform;
    target_transform.setIdentity();

    std::string target_frame_id(req.target_frame_id.data);
    geometry_msgs::PoseStamped target_pose_stamped, transformed_pose_stamped;

    target_pose_stamped.pose = req.target_pose;
    target_pose_stamped.header.frame_id = target_frame_id;

    transformed_pose_stamped.header.frame_id = robot.planning_frame;

    bool success = false;

    try {
        bool tf_success = tf_listener->waitForTransform(
            robot.planning_frame, target_frame_id, ros::Time(0),
            ros::Duration(WAIT_TIMEOUT));
        if (tf_success) {
            tf_listener->getLatestCommonTime(
                robot.planning_frame, target_frame_id,
                target_pose_stamped.header.stamp, NULL);

            tf_listener->transformPose(robot.planning_frame,
                                       target_pose_stamped,
                                       transformed_pose_stamped);

            transformed_pose_stamped.header.stamp = ros::Time::now();
            transformed_pose_stamped.header.frame_id = robot.planning_frame;

            ROS_INFO_STREAM("starting pose in target frame"
                            << target_pose_stamped);
            ROS_INFO_STREAM("Transformed pose to planning frame"
                            << transformed_pose_stamped);

            float velocity_scale = 1.0;
            if (req.velocity_scaling_factor.data) {
                velocity_scale = req.velocity_scaling_factor.data;
            }
            float max_planning_time = DEFAULT_PLANNING_TIME;
            if (req.max_planning_time.data) {
                max_planning_time = req.max_planning_time.data;
            }
            int max_attempts = MAX_ATTEMPTS;
            if (req.max_attempts.data) {
                max_attempts = req.max_attempts.data;
            }

            success = robot.moveTo(transformed_pose_stamped, max_attempts,
                                   MOVE_TIMEOUT, max_planning_time,
                                   velocity_scale);
        }

        //      geometry_msgs::PoseStamped target_pose;
        //      Eigen::Affine3d target_pose_eigen;
        //      tf::transformTFToEigen(target_transform, target_pose_eigen);
        //      tf::poseEigenToMsg(target_pose_eigen,target_pose.pose);
    } catch (tf::TransformException ex) {
        ROS_ERROR("%s", ex.what());
        res.success.data = false;
        return false;
    }

    res.success.data = success;
    return true;
}

bool move_robot_pose(moveit_lib::move_robot_pose::Request &req,
                     moveit_lib::move_robot_pose::Response &res) {
    bool flag = false;  // execution successful or not
    moveit_robot robot(req.move_group.data);

    printf("move_group requested: %s\n", req.move_group.data.c_str());
    printf("move group in robot: %s\n", robot.move_group.getName().c_str());

    try {
        geometry_msgs::PoseStamped target_pose;

        bool tf_success = tf_listener->waitForTransform(
            robot.planning_frame, req.target_pose.header.frame_id, ros::Time(0),
            ros::Duration(WAIT_TIMEOUT));

        if (tf_success) {
            tf_listener->getLatestCommonTime(
                robot.planning_frame, req.target_pose.header.frame_id,
                req.target_pose.header.stamp, NULL);
            tf_listener->transformPose(robot.planning_frame, req.target_pose,
                                       target_pose);

            ROS_INFO_STREAM("starting pose in target frame" << target_pose);
            pose_pub.publish(target_pose);
            ros::spinOnce();
            float velocity_scale = 1.0;
            if (req.velocity_scaling_factor.data) {
                velocity_scale = req.velocity_scaling_factor.data;
            }
            float max_planning_time = DEFAULT_PLANNING_TIME;
            if (req.max_planning_time.data) {
                max_planning_time = req.max_planning_time.data;
            }
            int max_attempts = MAX_ATTEMPTS;
            if (req.max_attempts.data) {
                max_attempts = req.max_attempts.data;
            }
            flag = robot.moveTo(target_pose, max_attempts, MOVE_TIMEOUT,
                                max_planning_time, velocity_scale);
        } else {
            ROS_WARN("Could not lookup transform.");
        }
    } catch (tf::TransformException ex) {
        ROS_ERROR("TransformException: %s", ex.what());
        res.success.data = false;
        return false;
    } catch (...) {
        ROS_ERROR("Unknown exception.");
        res.success.data = false;
        return false;
    }

    res.success.data = flag;
    return true;
}

bool move_robot_pose_constrained(
    moveit_lib::move_robot_pose_constrained::Request &req,
    moveit_lib::move_robot_pose_constrained::Response &res) {
    moveit_robot robot(req.move_group.data);

    bool success = false;
    try {
        geometry_msgs::PoseStamped target_pose;

        bool tf_success = tf_listener->waitForTransform(
            robot.planning_frame, req.target_pose.header.frame_id, ros::Time(0),
            ros::Duration(WAIT_TIMEOUT));

        if (tf_success) {
            tf_listener->getLatestCommonTime(
                robot.planning_frame, req.target_pose.header.frame_id,
                req.target_pose.header.stamp, NULL);
            tf_listener->transformPose(robot.planning_frame, req.target_pose,
                                       target_pose);

            float velocity_scale = 1.0;
            if (req.velocity_scaling_factor.data) {
                velocity_scale = req.velocity_scaling_factor.data;
            }

            success = robot.moveToOrientationConstraint(
                target_pose, req.qConstraint, req.link_name.data, MAX_ATTEMPTS,
                MOVE_TIMEOUT, DEFAULT_PLANNING_TIME, velocity_scale);
        }
    } catch (tf::TransformException ex) {
        ROS_ERROR("%s", ex.what());
        res.success.data = false;
        return false;
    }

    res.success.data = success;
    return true;
}

bool move_robot_pose_array(moveit_lib::move_robot_pose_array::Request &req,
                           moveit_lib::move_robot_pose_array::Response &res) {
    ROS_INFO_STREAM("moveToCartesianPath info in ros node\n\n");

    moveit_robot robot(req.move_group.data);
    double success_rate = false;

    try {
        std::vector<geometry_msgs::Pose> target_pose_vector;

        geometry_msgs::PoseStamped target_pose;
        // geometry_msgs::PoseArray target_pose_array;
        geometry_msgs::PoseStamped orig_pose;

        // target_pose_array.header.frame_id = robot.planning_frame;
        // target_pose_array.header.stamp = ros::Time::now();

        for (size_t i = 0; i < req.target_pose_array.poses.size(); i++) {
            orig_pose.header = req.target_pose_array.header;
            orig_pose.pose = req.target_pose_array.poses[i];

            // waiting for a common time for both fram  es
            bool tf_success = tf_listener->waitForTransform(
                robot.planning_frame, orig_pose.header.frame_id, ros::Time(0),
                ros::Duration(WAIT_TIMEOUT));

            // required to have a common time before looking up transform
            if (tf_success) {
                // get the common time for both tfs
                ros::Time common_time;
                tf_listener->getLatestCommonTime(robot.planning_frame,
                                                 orig_pose.header.frame_id,
                                                 common_time, NULL);

                orig_pose.header.stamp = common_time;

                tf_listener->transformPose(robot.planning_frame, orig_pose,
                                           target_pose);
                target_pose_vector.push_back(target_pose.pose);
            }
        }
        float velocity_scale = 1.0;
        if (req.velocity_scaling_factor.data) {
            velocity_scale = req.velocity_scaling_factor.data;
        }

        geometry_msgs::PoseArray vis_pose_array;
        vis_pose_array.poses = target_pose_vector;
        vis_pose_array.header.frame_id = robot.planning_frame;
        vis_pose_array.header.stamp = ros::Time::now();
        pose_array_pub.publish(vis_pose_array);

        float max_planning_time = DEFAULT_PLANNING_TIME;
        if (req.max_planning_time.data) {
            max_planning_time = req.max_planning_time.data;
        }
        int max_attempts = MAX_ATTEMPTS;
        printf("max_attmpts in moveit: %d", req.max_attempts.data);
        if (req.max_attempts.data) {
            max_attempts = req.max_attempts.data;
        }

        ROS_INFO("Attempting to plan a cartesian path");
        success_rate = robot.moveToCartesianPath(
            target_pose_vector, max_attempts, MOVE_TIMEOUT,
            max_planning_time, velocity_scale, 0.005, 0.0, req.min_path_completion.data);

    } catch (tf::TransformException ex) {
        ROS_ERROR("%s", ex.what());
        res.success.data = false;
        return false;
    }

    if (success_rate < req.min_path_completion.data) {
        ROS_WARN("moveToCartesianPath failed");
    }

    res.success.data = (success_rate > req.min_path_completion.data);

    return true;
}

bool move_robot_named(moveit_lib::move_robot_named::Request &req,
                      moveit_lib::move_robot_named::Response &res) {
    moveit_robot robot(req.move_group.data);
    bool success = false;
    try {
        float velocity_scale = 1.0;
        if (req.velocity_scaling_factor.data) {
            velocity_scale = req.velocity_scaling_factor.data;
        }
        float max_planning_time = DEFAULT_PLANNING_TIME;
        if (req.max_planning_time.data) {
            max_planning_time = req.max_planning_time.data;
        }
        int max_attempts = MAX_ATTEMPTS;
        if (req.max_attempts.data) {
            max_attempts = req.max_attempts.data;
        }
        // printf("max attempts: %d\n", max_attempts );
        success =
            robot.moveToNamed(req.named_pose.data, max_attempts, MOVE_TIMEOUT,
                              max_planning_time, velocity_scale);
    } catch (tf::TransformException ex) {
        ROS_ERROR("%s", ex.what());
        res.success.data = false;
        return false;
    }

    res.success.data = success;
    return true;
}

// The main procedure is here / starts up the ROS Node
int main(int argc, char **argv) {
    ros::init(argc, argv, "moveit_robot_node");

    tf_listener = new tf::TransformListener();

    ros::NodeHandle nh_("~");
    pose_pub =
        nh_.advertise<geometry_msgs::PoseStamped>("/moveit_target_pose", 0);
    pose_array_pub =
        nh_.advertise<geometry_msgs::PoseArray>("/moveit_target_pose_array", 0);

    ros::ServiceServer move_tf_service =
        nh_.advertiseService("/moveit_lib/move_robot_tf", &move_robot_tf);
    ros::ServiceServer move_named_service =
        nh_.advertiseService("/moveit_lib/move_robot_named", &move_robot_named);
    ros::ServiceServer move_pose_service =
        nh_.advertiseService("/moveit_lib/move_robot_pose", &move_robot_pose);
    ros::ServiceServer move_pose_constrained_service =
        nh_.advertiseService("/moveit_lib/move_robot_pose_constrained",
                             &move_robot_pose_constrained);
    ros::ServiceServer move_pose_array_service = nh_.advertiseService(
        "/moveit_lib/move_robot_pose_array", &move_robot_pose_array);

    ros::AsyncSpinner spinner(2);
    spinner.start();

    ROS_INFO_STREAM("moveit_lib services started!");
    ros::Rate rate(50);
    while (ros::ok()) {
        rate.sleep();
    }

    ROS_INFO_STREAM("moveit_lib services to be stopped!");

    return 0;
}
