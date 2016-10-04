#include <ros/ros.h>
#include <baxter_core_msgs/EndpointState.h>
#include <iostream>
#include <fstream>
#include <math.h>
#include <string.h>

void torque_callback(baxter_core_msgs::EndpointState::ConstPtr msg) {
    // Find the average torque
    double average_torque = (std::abs(msg->wrench.torque.x) +
      std::abs(msg->wrench.torque.y) + std::abs(msg->wrench.torque.z)) / 3.0;
    std::ofstream torque_file;
    torque_file.open("torque_outputs.txt", std::ios_base::app);
    torque_file << " ";
    torque_file << std::to_string(average_torque);
    torque_file.close();
    ROS_INFO("Torque output file updated in home directory.");
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "baxter_torque_subscriber");
    ros::NodeHandle nh;

    ros::Subscriber torque_subscriber = nh.subscribe<baxter_core_msgs::EndpointState>("/robot/limb/left/endpoint_state", 1, torque_callback);

    // Clear the torque output file
    std::ofstream torque_file;
    torque_file.open("torque_outputs.txt");
    torque_file.close();
    ROS_INFO("Torque output file cleared in home directory.");

    ros::Rate rate(5);
    while(ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }
    return 1;
}
