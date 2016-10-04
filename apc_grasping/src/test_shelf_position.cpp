/*
   Copyright 2016 Australian Centre for Robotic Vision
 */
 #include <tf/transform_listener.h>

 #include <moveit/move_group_interface/move_group.h>
 #include <moveit/planning_scene/planning_scene.h>
 #include <moveit/planning_scene_interface/planning_scene_interface.h>
 #include <moveit/planning_scene_monitor/planning_scene_monitor.h>
 #include <moveit/robot_model/robot_model.h>
 #include <moveit/robot_model_loader/robot_model_loader.h>
 #include <moveit/robot_state/robot_state.h>
 #include <moveit_lib/move_robot_named.h>
 #include <moveit_msgs/PlanningScene.h>

 #include <eigen_conversions/eigen_msg.h>
 #include <tf/transform_datatypes.h>
 #include <tf_conversions/tf_eigen.h>

 #include <math.h>
 #include <algorithm>

 #include <apc_msgs/GraspPose.h>
 #include <apc_msgs/SelectGraspFromCandidates.h>
 #include <apc_msgs/SelectGraspFromModel.h>
 #include <moveit_lib/move_robot_pose.h>
 #include <apc_grasping/Item.h>

ros::Publisher marker_array_pub;
planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
tf::Transform shelf_to_base_transform;
tf::TransformListener *tf_listener;

geometry_msgs::Pose rotate_pose(geometry_msgs::Pose input_pose,
                                double angle_offset,
                                Eigen::Vector3d unit_axis) {
    Eigen::Affine3d input_pose_eigen, input_pose_eigen_rotated;
    geometry_msgs::Pose output_pose;
    tf::poseMsgToEigen(input_pose, input_pose_eigen);

    Eigen::Affine3d rotation(Eigen::AngleAxisd(angle_offset, unit_axis));

    input_pose_eigen_rotated = input_pose_eigen * rotation;

    tf::poseEigenToMsg(input_pose_eigen_rotated, output_pose);

    return output_pose;
}

bool lookupTransform(std::string target_frame, std::string source_frame,
                     tf::Transform *output_transform) {

    // tf::Transform transform;
    tf::StampedTransform stampedtransform;
    bool tf_success = tf_listener->waitForTransform(
        target_frame, source_frame, ros::Time(0), ros::Duration(5.0));
    if (tf_success) {
        tf_listener->lookupTransform(target_frame, source_frame, ros::Time(0),
                                     stampedtransform);
        output_transform->setOrigin(stampedtransform.getOrigin());
        output_transform->setRotation(stampedtransform.getRotation());
        return true;
    } else {
        ROS_INFO("Can't Find Transform from %s to %s!", source_frame.c_str(),
                 target_frame.c_str());
        return false;
    }
}

bool transformPose(const geometry_msgs::PoseStamped &input_pose,
                   std::string target_frame_id,
                   geometry_msgs::PoseStamped *transformed_pose) {
    bool tf_success = tf_listener->waitForTransform(
        target_frame_id, input_pose.header.frame_id, ros::Time::now(),
        ros::Duration(5.0));
    if (tf_success) {

        tf_listener->transformPose(target_frame_id, input_pose,
                                   *transformed_pose);
        return true;
    } else {
        ROS_INFO("Can't Find Transform from %s to %s!",
                 input_pose.header.frame_id.c_str(), target_frame_id.c_str());
        return false;
    }
}

void transformPoseNoLookup(const geometry_msgs::Pose &input_pose,
                           const tf::Transform &transform,
                           geometry_msgs::Pose *transformed_pose) {
    Eigen::Affine3d input_pose_eigen, transform_eigen, transformed_pose_eigen;

    tf::transformTFToEigen(transform, transform_eigen);
    tf::poseMsgToEigen(input_pose, input_pose_eigen);

    transformed_pose_eigen = transform_eigen * input_pose_eigen;

    tf::poseEigenToMsg(transformed_pose_eigen, *transformed_pose);

    // THIS IS A HACK THAT WE DON'T KNOW HOW TO FIX
    // transformed_pose->orientation.x = -1 * transformed_pose->orientation.x;
    // transformed_pose->orientation.y = -1 * transformed_pose->orientation.y;
    // transformed_pose->orientation.z = -1 * transformed_pose->orientation.z;
    // transformed_pose->orientation.w = -1 * transformed_pose->orientation.w;
}


bool checkCollision(robot_state::RobotStatePtr kinematic_state,
                    planning_scene::PlanningScenePtr planning_scene,
                    std::string group_name) {
    collision_detection::CollisionRequest collision_request;
    collision_request.group_name = group_name;
    collision_request.contacts = true;
    collision_detection::CollisionResult collision_result;
    collision_result.clear();

    planning_scene->checkCollision(collision_request, collision_result,
                                   *kinematic_state);
    return collision_result.collision;
}

bool isStateValid(
    const planning_scene::PlanningScene *planning_scene,
    const kinematic_constraints::KinematicConstraintSet *constraint_set,
    robot_state::RobotState *state, const robot_state::JointModelGroup *group,
    const double *ik_solution) {
    state->setJointGroupPositions(group, ik_solution);
    state->update();
    return (!planning_scene ||
            !planning_scene->isStateColliding(*state, group->getName())) &&
           (!constraint_set || constraint_set->decide(*state).satisfied);
}

bool isStateValidSimple(const planning_scene::PlanningScene *planning_scene,
                        robot_state::RobotState *state,
                        const robot_state::JointModelGroup *group,
                        const double *ik_solution) {
    state->setJointGroupPositions(group, ik_solution);
    state->update();
    // return (!planning_scene ||
    //         !planning_scene->isStateColliding(*state, group->getName()));
    return !planning_scene->isStateColliding(*state, group->getName());
}

bool checkIKandCollision(const robot_state::JointModelGroup *jmg,
                         robot_state::RobotStatePtr state,
                         geometry_msgs::Pose *pose, int attempts) {
    kinematics::KinematicsQueryOptions options;

    boost::scoped_ptr<planning_scene_monitor::LockedPlanningSceneRO> ls;
    ls.reset(new planning_scene_monitor::LockedPlanningSceneRO(
        planning_scene_monitor_));

    std::string link_name = jmg->getLinkModelNames().back();

    const moveit::core::GroupStateValidityCallbackFn validity_fn = boost::bind(
        &isStateValidSimple,
        static_cast<const planning_scene::PlanningSceneConstPtr &>(*ls).get(),
        _1, _2, _3);

    // Can add tip link frame using RobotState.getLinkModel()
    bool found_ik =
        state->setFromIK(jmg, *pose, link_name, attempts, 0.0, validity_fn);

    // ROS_INFO_STREAM("Link Name = " << link_name);
    const Eigen::Affine3d &end_effector_pose =
        state->getGlobalLinkTransform(link_name);

    tf::poseEigenToMsg(end_effector_pose, *pose);

    return found_ik;
}

bool checkIK(const robot_state::JointModelGroup *jmg,
             robot_state::RobotStatePtr state, geometry_msgs::Pose pose,
             int attempts) {
    // Can add tip link frame using RobotState.getLinkModel()
    // std::string link_name = jmg->getLinkModelNames().back();
    bool ik = state->setFromIK(jmg, pose, attempts, 0.1);
    return (ik);
}


geometry_msgs::Pose translate_pose(geometry_msgs::Pose input_pose,
                                   Eigen::Vector3d offset) {
    Eigen::Affine3d input_pose_eigen, input_pose_eigen_translated;
    geometry_msgs::Pose output_pose;
    tf::poseMsgToEigen(input_pose, input_pose_eigen);

    Eigen::Translation3d translation(offset);

    input_pose_eigen_translated = input_pose_eigen * translation;

    tf::poseEigenToMsg(input_pose_eigen_translated, output_pose);

    return output_pose;
}

void deleteGraspMarkers(geometry_msgs::PoseArray poses) {
    visualization_msgs::MarkerArray pose_array_marker_delete;

    // Delete markers
    for (unsigned int i = 0; i < 20000; i++) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = poses.header.frame_id;
        marker.header.stamp = ros::Time::now();
        marker.ns = "poses";
        marker.id = i;
        // marker.type = visualization_msgs::Marker::ARROW;
        marker.action = visualization_msgs::Marker::DELETE;
        pose_array_marker_delete.markers.push_back(marker);
    }
    marker_array_pub.publish(pose_array_marker_delete);
    ROS_INFO("Deleted past markers");
    ros::WallDuration sleep_t(5.0);
    sleep_t.sleep();
}

void publishGraspMarkers(geometry_msgs::PoseArray poses, double marker_scale, bool success) {
    visualization_msgs::MarkerArray pose_marker_array;

    // Publish new markers
    for (unsigned int i = 0; i < poses.poses.size(); i++) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = poses.header.frame_id;
        marker.header.stamp = ros::Time::now();
        marker.ns = "poses";
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = std::abs(marker_scale);
        marker.scale.y = std::abs(marker_scale);
        marker.scale.z = std::abs(marker_scale);
        marker.color.a = 1.0;
        if (success == true) {
            marker.id = i;
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
        } else {
            marker.id = i + 8000;
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
        }
        //        marker.pose = poses.poses[i];
        marker.pose = poses.poses[i];
        pose_marker_array.markers.push_back(marker);
    }
    marker_array_pub.publish(pose_marker_array);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "test_shelf_position");
    ros::NodeHandle nh;

    ros::AsyncSpinner spinner(2);
    spinner.start();

    // Create pubisher to publish on moveit's planning scene
    ros::Publisher planning_scene_publisher =
        nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);

    tf_listener = new tf::TransformListener;
    tf::TransformListener tf_listener_;

    Eigen::Vector3d x_rotation(1, 0, 0);
    Eigen::Vector3d z_rotation(0, 0, 1);

    robot_model::RobotModelPtr kinematic_model;
    robot_state::RobotStatePtr robot_state;
    robot_model_loader::RobotModelLoader robot_model_loader(
        "robot_description");
    kinematic_model = robot_model_loader.getModel();


    // Create a monitored planning scene
    robot_state.reset(new robot_state::RobotState(kinematic_model));
    planning_scene_monitor_ =
        boost::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
            "robot_description");

    planning_scene_monitor_->startStateMonitor("/joint_states",
                                               "/attached_collision_object");
    planning_scene_monitor_->startSceneMonitor("/planning_scene");
    planning_scene_monitor_->startWorldGeometryMonitor();

    ros::WallDuration sleep_t(0.1);

    // Get the initial pose of the shelf
    double shelf_x, shelf_y, shelf_z, shelf_roll, shelf_pitch, shelf_yaw;
    nh.param("test_shelf/shelf_x", shelf_x, 1.5);
    nh.param("test_shelf/shelf_y", shelf_y, 0.0);
    nh.param("test_shelf/shelf_z", shelf_z, -0.87);
    nh.param("test_shelf/shelf_roll", shelf_roll, 90.0);
    nh.param("test_shelf/shelf_pitch", shelf_pitch, 45.0);
    nh.param("test_shelf/shelf_yaw", shelf_yaw, 90.0);
    shelf_x += 0.02465;
    shelf_y += 0.21965;

    // Publisher for pose markers
    marker_array_pub =
        nh.advertise<visualization_msgs::MarkerArray>("/selected_grasp_markers", 0);

    // Get the current pose of teh shelftf::TransformListener tf_listener_;
    // bool tf_success = tf_listener_.waitForTransform("base", "pod_lowres", ros::Time::now(),
    //     ros::Duration(5.0));
    // if (tf_success) {
    //     ro
    // }
    geometry_msgs::PoseStamped current_pose, current_pose_base, current_rotated_pose;
    current_pose.header.frame_id = "pod_lowres";
    current_rotated_pose.header.frame_id = "pod_lowres";
    current_pose_base.header.frame_id = "base";

    // Initial offset of pose to match with the shelfs corner
    double x_offset, y_offset, z_offset;
    nh.param("test_shelf/x_offset", x_offset, 1.0);
    nh.param("test_shelf/y_offset", y_offset, 1.0);
    nh.param("test_shelf/z_offset", z_offset, 1.0);

    // get the marker length in x, y and z
    int x_length, y_length, z_length, ik_attemps;
    nh.param("test_shelf/x_length", x_length, 20);
    nh.param("test_shelf/y_length", y_length, 20);
    nh.param("test_shelf/z_length", z_length, 20);

    // get the number of attempts to use in inverse kinematics
    nh.param("test_shelf/ik_attemps", ik_attemps, 2);

    // Get the shelf dimensions
    double shelf_width, shelf_height, shelf_depth;
    nh.param("test_shelf/shelf_width", shelf_width, 0.8);
    nh.param("test_shelf/shelf_height", shelf_height, 0.99);
    nh.param("test_shelf/shelf_depth", shelf_depth, 0.4);

    // Get the testing distance of the shelf and the step size
    double testing_distance, step_size;
    nh.param("test_shelf/testing_distance", testing_distance, 1.0);
    nh.param("test_shelf/step_size", step_size, 0.1);

    // Get whether to visualize the markres or not
    int visualize;
    nh.param("test_shelf/visualize", visualize, 0);

    // Get the move group
    std::string move_group;
    nh.param<std::string>("test_shelf/move_group", move_group, "left_arm");

    std::vector<double> scores;
    double x_distance, y_distance;
    double successful_amount, unsuccessful_amount, score;
    // Loop to test the shelf position
    double tests, i, j, k;
    for (tests = 0; tests <= testing_distance; tests += step_size) {
        // Find transform from shelf fram to base frame
        lookupTransform("base", "pod_lowres", &shelf_to_base_transform);

        geometry_msgs::PoseArray successful_poses, unsuccessful_poses;
        successful_poses.header.frame_id = "pod_lowres";
        unsuccessful_poses.header.frame_id = "pod_lowres";
        // Update the position of the shelf collision object
        moveit_msgs::PlanningScene planning_scene;
        planning_scene.is_diff = true;
        x_distance = shelf_x + (tests * cos(45 * (M_PI / 180)));
        y_distance = shelf_y + (tests * cos(45 * (M_PI / 180)));
        Item pod_lowres(x_distance, y_distance, shelf_z, shelf_roll, shelf_pitch, shelf_yaw,
            "pod_lowres", &planning_scene);
        pod_lowres.addCollisionItem();
        sleep_t.sleep();
        planning_scene_publisher.publish(planning_scene);
        sleep_t.sleep();
        ROS_INFO_STREAM("Current x position:  " << x_distance);
        ROS_INFO_STREAM("Current y position:  " << y_distance);
        ROS_INFO_STREAM("Current Distance from shoulder joint:  " << ((x_distance - 0.02465) / cos(45 * (M_PI / 180))));
        for (i = 0; i <= (shelf_width + 0.01); i += (shelf_width / (x_length - 1))) {
            current_pose.pose.position.x = x_offset - i;
            for(j = 0; j <= (shelf_height + 0.01); j += (shelf_height / (y_length - 1))) {
                current_pose.pose.position.y = y_offset - j;
                for (k = 0; k <= (shelf_depth + 0.01); k += (shelf_depth / (z_length - 1))) {
                    current_pose.pose.position.z = z_offset + k;

                    // Rotate the pose
                    current_rotated_pose.pose = rotate_pose(current_pose.pose, (-90 * (M_PI / 180)), x_rotation);
                    current_rotated_pose.pose = rotate_pose(current_rotated_pose.pose, (-90 * (M_PI / 180)), z_rotation);

                    // Transform current pose to base frame
                    // transformPose(current_rotated_pose, "base", &current_pose_base);
                    transformPoseNoLookup(current_rotated_pose.pose, shelf_to_base_transform, &current_pose_base.pose);

                    // Check IK and Collisions for current pose
                    if (checkIKandCollision(
                            kinematic_model->getJointModelGroup(move_group),
                            robot_state, &current_pose_base.pose, ik_attemps)) {
                        // ROS_INFO("lsjd;lkfjs");
                        successful_poses.poses.push_back(current_rotated_pose.pose);
                    } else {
                        unsuccessful_poses.poses.push_back(current_rotated_pose.pose);
                    }
                }
            }
        }
        successful_amount = successful_poses.poses.size();
        unsuccessful_amount = unsuccessful_poses.poses.size();
        score = successful_amount / (successful_amount + unsuccessful_amount);
        ROS_INFO_STREAM("Number of successful poses:  " << successful_amount);
        ROS_INFO_STREAM("Number of unsuccessful poses:  " << unsuccessful_amount);
        ROS_INFO_STREAM("Overall score:  " << score);
        std::cout << std::endl;
        scores.push_back(score);

        if (visualize == 1) {
            double marker_scale;
            nh.param("test_shelf/marker_scale", marker_scale, 0.02);
            deleteGraspMarkers(successful_poses);
            publishGraspMarkers(successful_poses, marker_scale, true);
            publishGraspMarkers(unsuccessful_poses, marker_scale, false);
            ROS_INFO("Test Marker Published");
        }
    }

    // Determine the optimal distance to position the shelf
    auto best_score = std::max_element(std::begin(scores), std::end(scores));
    int best_score_idx = std::distance(std::begin(scores), best_score);
    double best_distance = (shelf_x - 0.02465 + (best_score_idx * step_size * cos(45 * (M_PI / 180)))) / cos(45 * (M_PI / 180));
    ROS_INFO_STREAM("BETS SCORE INDEX:   " << best_score_idx);
    ROS_INFO_STREAM("BEST DISTANCE FOR OPTIMAL GRASPING:   " << best_distance);
    // double best_distance = shelf_x + best_score_idx;

    // Loop forever
    while(1) {}
    return 0;
}
