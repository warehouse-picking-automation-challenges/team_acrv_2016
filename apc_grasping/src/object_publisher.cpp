/*
Copyright 2016 Australian Centre for Robotic Vision
*/
#include <apc_grasping/Item.h>

// Global variable to store the item ids
std::vector <std::string> item_ids;
// Global variable to store the poses of the items
std::vector <geometry_msgs::Pose> item_poses;

// Callback function to determine poses of collision objects in the scene
void collision_object_callback(
    const moveit_msgs::PlanningScene::ConstPtr msg) {
    // Clear the global variables
    item_ids.clear();
    item_poses.clear();
    std::vector<moveit_msgs::CollisionObject> collision_objects =
        msg->world.collision_objects;
    for(unsigned int i = 0; i < collision_objects.size(); i++) {
        ROS_INFO_STREAM("Item ID: " << collision_objects[i].id);
        item_ids.push_back(collision_objects[i].id);
        item_poses.push_back(collision_objects[i].mesh_poses[0]);
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "object_publisher");
    ros::NodeHandle nh;
    ros::Subscriber collision_object_subscriber =
        nh.subscribe<moveit_msgs::PlanningScene>("/planning_scene", 1,
        collision_object_callback);
    moveit::planning_interface::PlanningSceneInterface
        planning_scene_interface;

    ROS_INFO("TEST 1");
    // item_ids = planning_scene_interface.getKnownObjectNames(false);
    // pose_map = planning_scene_interface.getObjectPoses(item_ids);
    ros::Rate rate(5);
    while(ros::ok()) {
        for(unsigned int i = 0; i < item_ids.size(); i++) {
            // ROS_INFO_STREAM("Pose of " << item_ids[i] << " is:\n" << item_pose);
            // Get pose of eraser object
            // eraser_pose = pose_map["expo_dry_erase_board_eraser"];
            Item item(item_poses[i], item_ids[i]);
            rate.sleep();
            // static Item eraser(eraser_pose, "expo_dry_erase_board_eraser", nh);

            // Update pose and broadcast the tranform
            // item.setPose(item_pose);
            item.broadcastTf();
            rate.sleep();
        }
        ros::spinOnce();
    }
    ros::shutdown();
    return 0;
}
