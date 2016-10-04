/*
Copyright 2016 Australian Centre for Robotic Vision
*/
#include <apc_grasping/Item.h>


int main(int argc, char** argv) {
    ros::init(argc, argv, "create_items");

    ros::NodeHandle nh;

    ros::AsyncSpinner spinner(2);
    spinner.start();

    // Create pubisher to publish on moveit's planning scene
    ros::Publisher planning_scene_publisher =
        nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);

    while(planning_scene_publisher.getNumSubscribers() < 1)
    {
      ros::WallDuration sleep_t(0.5);
      sleep_t.sleep();
    }
    ros::WallDuration sleep_t(0.1);
    sleep_t.sleep();
    moveit_msgs::PlanningScene planning_scene;
    planning_scene.is_diff = true;

    Item pod_lowres(1.5, 0, -0.87, 90, 0, 90,
        "pod_lowres", &planning_scene);
    // Item kleenex_tissue_box(0.9, 0.6, 0.2, 0, 0, 0,
    //     "kleenex_tissue_box", &planning_scene);
    // Item platinum_pets_dog_bowl(0.9, 0.4, 0.2, 0, 0, 0,
    //     "platinum_pets_dog_bowl", &planning_scene);
    // Item fitness_gear_3lb_dumbbell(0.9, 0.2, 0.2, 0, 0, 0,
    //     "fitness_gear_3lb_dumbbell", &planning_scene);
    // Item rawlings_baseball(0.9, 0, 0.2, 0, 0, 0,
    //     "rawlings_baseball", &planning_scene);

    // Add the collision objects the planning scene
    pod_lowres.addCollisionItem();
    sleep_t.sleep();
    // kleenex_tissue_box.addCollisionItem();
    // sleep_t.sleep();
    // platinum_pets_dog_bowl.addCollisionItem();
    // sleep_t.sleep();
    // fitness_gear_3lb_dumbbell.addCollisionItem();
    // sleep_t.sleep();
    // rawlings_baseball.addCollisionItem();
    // sleep_t.sleep();


    // Publish the current planning scene
    planning_scene_publisher.publish(planning_scene);
    sleep_t.sleep();

    // while(ros::ok()) {}
    // ros::shutdown();
    return 0;
}
