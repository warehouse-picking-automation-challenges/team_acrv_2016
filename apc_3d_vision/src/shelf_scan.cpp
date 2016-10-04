#include "shelf_scan.h"

shelf_scanner::shelf_scanner(std::string move_group) {
    // initialise various
    init();

    tf::StampedTransform target_transform;
    tf::StampedTransform orig_transform;

    bool tf_success = tf_listener.waitForTransform(
        "/base", "camera_link", ros::Time(0), ros::Duration(10));
    ROS_INFO_STREAM(tf_success);
    if (tf_success) {
        tf_listener.getLatestCommonTime("/base", "camera_link", common_time,
                                        NULL);
        tf_listener.lookupTransform("/base", "camera_link", common_time,
                                    target_transform);
    }

    tf_success = tf_listener.waitForTransform("shelf", "camera_link",
                                              ros::Time(0), ros::Duration(10));
    ROS_INFO_STREAM(tf_success);
    if (tf_success) {
        tf_listener.getLatestCommonTime("shelf", "camera_link", common_time,
                                        NULL);
        tf_listener.lookupTransform("shelf", "camera_link", common_time,
                                    orig_transform);
    }

    shelf_pose.header.frame_id = "/base";
    shelf_pose.pose.position.x = target_transform.getOrigin()[0];
    shelf_pose.pose.position.y = target_transform.getOrigin()[1];
    shelf_pose.pose.position.z = target_transform.getOrigin()[2];

    tf::Quaternion tfQuaternion = target_transform.getRotation();

    tf::quaternionTFToMsg(tfQuaternion, shelf_pose.pose.orientation);

    orig_pose.header.frame_id = "shelf";
    orig_pose.pose.position.x = orig_transform.getOrigin()[0];
    orig_pose.pose.position.y = orig_transform.getOrigin()[1];
    orig_pose.pose.position.z = orig_transform.getOrigin()[2];

    tfQuaternion = orig_transform.getRotation();

    tf::quaternionTFToMsg(tfQuaternion, orig_pose.pose.orientation);

    // set move_group on moveit_lib messages
    pose_srv.request.move_group.data = move_group;
    pose_array_srv.request.move_group.data = move_group + "_cartesian";
    pose_array_srv.request.velocity_scaling_factor.data = 0.05;
}

shelf_scanner::shelf_scanner(std::string move_group, std::string bin_id) {
    // initialise various
    init();
    tf::StampedTransform target_transform;
    geometry_msgs::PoseStamped target_pose;

    double shelf_width = shelf_layout_[bin_id]["bin_width"];
    double shelf_height = shelf_layout_[bin_id]["bin_height"];

    orig_pose.header.frame_id = bin_id;
    orig_pose.pose.position.y = -(shelf_width / 2);
    // tf::Quaternion quat(0, 0, -M_PI / 2);
    if (bin_id.compare("bin_A") == 0 || bin_id.compare("bin_B") == 0 ||
        bin_id.compare("bin_C") == 0) {
        orig_pose.pose.position.x = HEIGHT_ABOVE_SHELF[0];

        orig_pose.pose.position.z = OFFSET_FROM_SHELF[0];
        tf::Quaternion quat2(ANGLE[0], 0, 0);
        tf::quaternionTFToMsg(quat2, orig_pose.pose.orientation);
    } else {
        orig_pose.pose.position.x = HEIGHT_ABOVE_SHELF[1];

        orig_pose.pose.position.z = OFFSET_FROM_SHELF[1];
        tf::Quaternion quat2(ANGLE[1], 0, 0);
        tf::quaternionTFToMsg(quat2, orig_pose.pose.orientation);
    }

    bool tf_success = tf_listener.waitForTransform(
        "/base", bin_id, ros::Time(0), ros::Duration(10));
    ROS_INFO_STREAM(tf_success);
    if (tf_success) {
        tf_listener.getLatestCommonTime("/base", bin_id, common_time, NULL);
        orig_pose.header.stamp = common_time;

        tf_listener.transformPose("/base", orig_pose, shelf_pose);
    }

    // set move_group on moveit_lib messages
    pose_srv.request.move_group.data = move_group;
    pose_array_srv.request.move_group.data = move_group + "_cartesian";
    pose_array_srv.request.velocity_scaling_factor.data = 0.05;
    pose_array_srv.request.min_path_completion.data = 0.75;

    ROS_INFO_STREAM("End init");
}

void shelf_scanner::setShelfRadius(float scan_radius){
    radius = scan_radius;
}

void shelf_scanner::init() {
    // set up moveit_lib service clients
    pose_client = nh_.serviceClient<moveit_lib::move_robot_pose>(
        "moveit_lib/move_robot_pose");
    pose_array_client = nh_.serviceClient<moveit_lib::move_robot_pose_array>(
        "moveit_lib/move_robot_pose_array");

    nh_.getParam("/shelf_layout", shelf_layout_);

    // setup publisher for scan waypoints, mainly for debugging
    vis_pub =
        nh_.advertise<geometry_msgs::PoseArray>("/shelf_scan/waypoints", 0);
    vis_pub2 =
        nh_.advertise<geometry_msgs::PoseStamped>("/shelf_scan/init_pose", 0);

    // setup kinfu advertisers
    reset_pub = nh_.advertise<std_msgs::Empty>("/ros_kinfu/reset", 10);
    pause_pub = nh_.advertise<std_msgs::Empty>("/ros_kinfu/pause", 10);
    unpause_pub = nh_.advertise<std_msgs::Empty>("/ros_kinfu/unpause", 10);

    ros::Publisher points_pub;

    // temporary variables
    start_q = {0.0, 0.707, 0.0, 0.707};
    delta_xyz = {0.10, 0.10, 0.10};
    scan_offset = {0.35, 0.0, 0.08};
    // radius = 0.075;
    distance = 0.5;
    segments = 4;
}

// currently not used, unlikely to be
void shelf_scanner::reinit(std::string move_group,
                           geometry_msgs::PoseStamped initial_pose) {
    shelf_pose = initial_pose;
    pose_srv.request.move_group.data = move_group;
    pose_array_srv.request.move_group.data = move_group;
}

// generate desired path, expand later for various different path options
void shelf_scanner::generatePath() {
    // createSimplePath(start_q, delta_xyz,scan_offset);
    createConePath(radius, distance, segments);
}

void shelf_scanner::createConePath(float scan_radius, float distance, int segments) {

    float arc_theta = 2 * M_PI / segments;
    float current_theta = 0.0;
    geometry_msgs::Pose pose;
    waypoints.header.frame_id = shelf_pose.header.frame_id;
    waypoints.header.stamp = ros::Time::now();
    while (current_theta < 2 * M_PI) {
        pose = orig_pose.pose;

        float dy = scan_radius * sin(current_theta);
        float dz = scan_radius * cos(current_theta);

        float theta_y = atan2(dy, distance);
        float theta_z = atan2(dz, distance);

        pose.position.y += dy;
        pose.position.x += dz;
        // tf::Quaternion initial_orientation;
        // tf::quaternionMsgToTF(orig_pose.pose.orientation,
        // initial_orientation);
        // tf::Quaternion quat(-theta_z, 0, -theta_y);
        //
        // initial_orientation *= quat;
        // tf::quaternionTFToMsg(initial_orientation, pose.orientation);
        pose.orientation = orig_pose.pose.orientation;

        current_theta += arc_theta;
        geometry_msgs::PoseStamped temp_pose;
        geometry_msgs::PoseStamped temp_pose2;
        temp_pose.pose = pose;
        temp_pose.header.stamp = common_time;
        temp_pose.header.frame_id = orig_pose.header.frame_id;

        tf_listener.transformPose(shelf_pose.header.frame_id, temp_pose,
                                  temp_pose2);

        waypoints.poses.push_back(temp_pose2.pose);
        // waypoints.poses.push_back(shelf_pose.pose);
    }
    waypoints.poses.push_back(shelf_pose.pose);
}

// temporary waypoint generator
bool shelf_scanner::createSimplePath(Eigen::Vector4d start_q_eigen,
                                     Eigen::Vector3d delta_xyz,
                                     Eigen::Vector3d scan_offset) {
    Eigen::Vector3d scan_xyz;

    geometry_msgs::Quaternion start_q, q_left, q_right, q_up, q_down;

    geometry_msgs::Pose scan_pose;

    double angle = 8 * M_PI / 180;

    start_q.x = start_q_eigen[0];
    start_q.y = start_q_eigen[1];
    start_q.z = start_q_eigen[2];
    start_q.w = start_q_eigen[3];

    q_left.x = cos(-angle);
    q_left.y = sin(-angle);
    q_left.z = 0.0;
    q_left.w = 0.0;
    q_up.x = cos(-angle);
    q_up.y = 0.0;
    q_up.z = sin(-angle);
    q_up.w = 0.0;
    q_right.x = cos(angle);
    q_right.y = sin(angle);
    q_right.z = 0.0;
    q_right.w = 0.0;
    q_down.x = cos(angle);
    q_down.y = 0.0;
    q_down.z = sin(angle);
    q_down.w = 0.0;

    scan_pose = shelf_pose.pose;

    waypoints.header.stamp = ros::Time::now();
    waypoints.header.frame_id = shelf_pose.header.frame_id;
    // Waypoint scan left
    scan_pose.position.y += delta_xyz[1];
    // scan_pose.orientation = q_left;
    waypoints.poses.push_back(scan_pose);

    // Waypoint scan up
    scan_pose.position.y -= delta_xyz[1];
    scan_pose.position.z += delta_xyz[2];
    // scan_pose.orientation = q_up;
    waypoints.poses.push_back(scan_pose);

    // Waypoint scan right
    scan_pose.position.z -= delta_xyz[2];
    scan_pose.position.y -= delta_xyz[1];
    // scan_pose.orientation = q_right;
    waypoints.poses.push_back(scan_pose);

    // scan down
    scan_pose.position.y += delta_xyz[1];
    scan_pose.position.z -= delta_xyz[2];
    // scan_pose.orientation = q_down;
    waypoints.poses.push_back(scan_pose);

    // Waypoint back to start;
    scan_pose.position.z += delta_xyz[2];
    // scan_pose.orientation = start_q;
    waypoints.poses.push_back(scan_pose);

    return true;
}

bool shelf_scanner::execute() {
    // send moveit_lib move to pose command
    pose_srv.request.target_pose = shelf_pose;
    vis_pub2.publish(orig_pose);

    bool success = pose_client.call(pose_srv);

    // if pose not successful, throw exception
    if (!pose_srv.response.success.data || !success) {
        ROS_INFO_STREAM(pose_srv.response.success.data);
        return false;
    }

    std::this_thread::sleep_for (std::chrono::seconds(3));

    // unpause and reset kinfu
    unpause_pub.publish(empty_msg);
    reset_pub.publish(empty_msg);

    pose_array_srv.request.target_pose_array = waypoints;
    vis_pub.publish(waypoints);
    ROS_INFO_STREAM("Starting to scan shelf.");
    success = pose_array_client.call(pose_array_srv);

    if (!pose_array_srv.response.success.data || !success) {
        ROS_INFO_STREAM("Pose array failed");
        return false;
    }
    //
    // ROS_INFO("Finished scanning shelf.");
    //
    // // pause kinfu
    pause_pub.publish(empty_msg);

    // clear waypoints
    waypoints.poses.clear();

    return true;
}
