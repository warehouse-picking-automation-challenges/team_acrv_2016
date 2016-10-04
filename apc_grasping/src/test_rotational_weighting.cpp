#include <ros/ros.h>
#include <math.h>
#include <string.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>


ros::Publisher marker_array_pub;

typedef enum indexes {
  UNIT_X_IDX = 0,
  UNIT_Y_IDX = 1,
  UNIT_Z_IDX = 2,
} UNIT_AXIS_INDEX;

double angle_between_pose(geometry_msgs::Pose grasp_pose,
                          uint8_t grasp_axis_idx, uint8_t base_axis_idx) {
  Eigen::Affine3d grasp_pose_eigen;
  tf::poseMsgToEigen(grasp_pose, grasp_pose_eigen);

  Eigen::Vector3d grasp_unit_axis =
      grasp_pose_eigen.rotation().col(grasp_axis_idx);

  // angle = acos (u.v / norm(u).norm(v) )
  // dot product of vector and unit vector is just the index of the vector
  // u.x = u[0], u.y = u[1], u.z = u[2]
  // norm(u).norm(z) = norm(u)*1 = norm(u)
  return acos(grasp_unit_axis[base_axis_idx] / grasp_unit_axis.norm());
}

void deleteGraspMarkers() {
    visualization_msgs::MarkerArray pose_array_marker_delete;

    // Delete markers
    for (unsigned int i = 0; i < 10000; i++) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "/base";
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

void publishGraspMarkers(geometry_msgs::PoseArray poses, std::vector<double> scores, double marker_scale) {
    visualization_msgs::MarkerArray pose_marker_array;

    // Publish new markers
    for (unsigned int i = 0; i < poses.poses.size(); i++) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = poses.header.frame_id;
        marker.header.stamp = ros::Time::now();
        marker.ns = "poses";
        marker.type = visualization_msgs::Marker::ARROW;
        marker.action = visualization_msgs::Marker::ADD;
        // marker.scale.x = std::abs(marker_scale * scores[i]);
        marker.scale.x = std::abs(marker_scale);
        marker.scale.y = marker_scale / 10.0;
        marker.scale.z = marker_scale / 10.0;
        marker.color.a = 1.0;
        marker.id = i;

        auto max_score = std::max_element(std::begin(scores), std::end(scores));
        auto min_score = std::min_element(std::begin(scores), std::end(scores));
        marker.color.r = 0.0;
        marker.color.g = (scores[i] - *min_score) / (*max_score - *min_score);
        marker.color.b = 0.0;
        //        marker.pose = poses.poses[i];
        marker.pose = poses.poses[i];
        pose_marker_array.markers.push_back(marker);
    }
    marker_array_pub.publish(pose_marker_array);
}

geometry_msgs::Pose createPose(double x, double y, double z, double roll, double pitch, double yaw) {
  geometry_msgs::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;
  pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll * (M_PI / 180.0), pitch * (M_PI / 180.0), yaw * (M_PI / 180.0));
  return pose;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "test_rotational_weighting");
    ros::NodeHandle nh;

    // Publisher for pose markers
    marker_array_pub =
        nh.advertise<visualization_msgs::MarkerArray>("/selected_grasp_markers", 0);

    double x, y, z, roll, pitch, yaw;
    x = 0;
    y = 0;
    z = 0;
    roll = 0;
    pitch = 0;
    yaw = 0;

    // Get values from parameter server
    int row, col;
    double angle_diff, marker_scale, square_size;
    std::string move_group;
    nh.param("test_rotation/row", row, 10);
    nh.param("test_rotation/col", col, 10);
    nh.param("test_rotation/angle_diff", angle_diff, 10.0);
    nh.param("test_rotation/marker_scale", marker_scale, 0.25);
    nh.param("test_rotation/square_size", square_size, 0.25);
    nh.param<std::string>("test_rotation/move_group", move_group, "left_arm");


    std::vector<double> scores;
    // Create testing pose's
    geometry_msgs::PoseArray poses;
    poses.header.frame_id = "/base";
    for (unsigned int i = 1; i <= row; i++) {
      z = 0.0;
      pitch = 0;
      if ((i % 2 == 0)) {
        y = std::abs(y) + square_size / row;
        yaw = -1 * std::abs(yaw) - 90.0 / (row / 2);
        // x = (i - 1) * 0.05;
      } else {
        y = y * -1;
        yaw = yaw * -1;
      }
      for (unsigned int j = 1; j <= col; j++) {
        geometry_msgs::Pose pose;
        if ((j % 2 == 0)) {
          z = std::abs(z) + square_size / col;
          pitch = std::abs(pitch) + 90.0 / (col / 2);
          // x += 0.05;
        } else {
          z = z * -1;
          pitch = pitch * -1;
        }
        pose = createPose(x, y, z, roll, pitch, yaw);

        // Determine rotational score
        double angle_between_x_and_z = angle_between_pose(pose, UNIT_X_IDX, UNIT_Z_IDX);
        double angle_between_x_and_y = angle_between_pose(pose, UNIT_X_IDX, UNIT_Y_IDX);

        double rotational_score;
        if (move_group == "left_arm") {
            rotational_score = (sin(angle_between_x_and_z) + sin(angle_between_x_and_y)) / 2.0;
        } else {
            rotational_score = std::max(-cos(angle_between_x_and_z), std::abs(cos(angle_between_x_and_y) * 0.5));
        }

        poses.poses.push_back(pose);
        scores.push_back(rotational_score);
      }
    }
    deleteGraspMarkers();
    publishGraspMarkers(poses, scores, marker_scale);
    ROS_INFO("Grasp markers published.");

    ros::Rate rate(5);
    ros::spinOnce();
    rate.sleep();
    return 1;
}
