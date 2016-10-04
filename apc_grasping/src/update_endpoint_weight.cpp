#include <ros/ros.h>
#include <baxter_core_msgs/EndEffectorCommand.h>
#include <string.h>
#include <apc_msgs/UpdateEndpointWeight.h>
#include <std_srvs/Empty.h>

ros::Publisher weight_pub;

std::string which_arm;

bool deleteEndpointWeight (std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
    // Create end effector command message to publish
    baxter_core_msgs::EndEffectorCommand end_effector_command;
    end_effector_command.id = 65537; // Rethink suction cup gripper
    end_effector_command.command = "configure";

    if (which_arm == "left_arm") {
      end_effector_command.args = "{ \"object\":{ \"name\": \"left_object_mass\", \"inertial\": { \"mass\": { \"value\": 0.0 }, \"origin\": { \"xyz\": [0.0, 0.0, 0.0] }, \"inertia\": { \"ixx\": 0.0, \"ixy\": 0.0, \"ixz\": 0.0, \"iyy\": 0.0, \"iyz\": 0.0, \"izz\": 0.0 } } } }}";
    } else if (which_arm == "right_arm") {
      end_effector_command.args = "{ \"object\":{ \"name\": \"right_object_mass\", \"inertial\": { \"mass\": { \"value\": 0.0 }, \"origin\": { \"xyz\": [0.0, 0.0, 0.0] }, \"inertia\": { \"ixx\": 0.0, \"ixy\": 0.0, \"ixz\": 0.0, \"iyy\": 0.0, \"iyz\": 0.0, \"izz\": 0.0 } } } }}";
    }
    // Publish the message
    weight_pub.publish(end_effector_command);
    return true;
}

bool updateEndpointWeight(
    apc_msgs::UpdateEndpointWeight::Request &req,
    apc_msgs::UpdateEndpointWeight::Response &res) {

    ros::NodeHandle nh;

    // Get the weight of the item
    double item_weight;
    std::string item_id = req.item_id;
    if (!nh.getParam("item_weights/" + item_id, item_weight)) {
        ROS_INFO_STREAM("Item " << item_id << " does not have any weight associated with it.");
        res.success.data = true;
        return false;
    }
    std::string item_weight_str = std::to_string(item_weight);

    // Create end effector command message to publish
    baxter_core_msgs::EndEffectorCommand end_effector_command;
    end_effector_command.id = 65537; // Rethink suction cup gripper
    end_effector_command.command = "configure";

    if (which_arm == "left_arm") {
      end_effector_command.args = "{ \"object\":{ \"name\": \"left_object_mass\", \"inertial\": { \"mass\": { \"value\": " + item_weight_str + " }, \"origin\": { \"xyz\": [0.0, 0.0, 0.0] }, \"inertia\": { \"ixx\": 0.0, \"ixy\": 0.0, \"ixz\": 0.0, \"iyy\": 0.0, \"iyz\": 0.0, \"izz\": 0.0 } } } }}";
    } else if (which_arm == "right_arm") {
      end_effector_command.args = "{ \"object\":{ \"name\": \"right_object_mass\", \"inertial\": { \"mass\": { \"value\": " + item_weight_str + " }, \"origin\": { \"xyz\": [0.0, 0.0, 0.0] }, \"inertia\": { \"ixx\": 0.0, \"ixy\": 0.0, \"ixz\": 0.0, \"iyy\": 0.0, \"iyz\": 0.0, \"izz\": 0.0 } } } }}";
    }

    // Publish the message
    weight_pub.publish(end_effector_command);
    res.success.data = true;
    return true;
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "update_endpoint_weight");

    ros::NodeHandle nh;

    nh.param("/apc_global/which_arm", which_arm, std::string("left_arm"));
    // Create publisher to change end effector weight
    if (which_arm == "left_arm") {
      weight_pub = nh.advertise<baxter_core_msgs::EndEffectorCommand>("/robot/end_effector/left_gripper/command", 1);
    } else if (which_arm == "right_arm") {
        weight_pub = nh.advertise<baxter_core_msgs::EndEffectorCommand>("/robot/end_effector/left_gripper/command", 1);
      }

    // Advertise Services
    ros::ServiceServer weightService = nh.advertiseService(
            "/apc_grasping/update_endpoint_weight", &updateEndpointWeight);

    ros::ServiceServer deleteWeightService = nh.advertiseService(
            "/apc_grasping/delete_endpoint_weight", &deleteEndpointWeight);

    ROS_INFO("update_endpoint_weight service started.");

    // Loop forever
    ros::spin();

    return 0;
}
