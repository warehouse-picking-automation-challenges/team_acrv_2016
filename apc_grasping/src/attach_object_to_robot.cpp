/*
Copyright 2016 Australian Centre for Robotic Vision
*/

#include <geometry_msgs/Pose.h>
#include <ros/ros.h>

#include <moveit_msgs/PlanningScene.h>

#include <moveit/move_group_interface/move_group.h>

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/GetStateValidity.h>
// #include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_state/robot_state.h>

#include <apc_msgs/AttachObjectCollisionModel.h>
#include <apc_msgs/DetachObjectCollisionModel.h>

// adapted from from ROS Planning Scene tutorial on MoveIt
// http://docs.ros.org/indigo/api/pr2_moveit_tutorials/html/planning/src/doc/planning_scene_ros_api_tutorial.html

#define EPSILON 0.001  // 1mm offset

class CollisionInterface {
   private:
    // Definions, are provided by the constructor and the service call!
    std::string link_to_attach_to_;
    std::string planning_scene_topic_;

    ros::Publisher planning_scene_publisher_;
    ros::NodeHandle nh_;

   public:
    // Constuctor
    CollisionInterface() {
        ROS_INFO("[Constructor] we need a node handle at least...");
        exit(1);
    }

    CollisionInterface(ros::NodeHandle &nh, std::string scene_topic) {
        nh_ = nh;
        planning_scene_topic_ = scene_topic;
        createPublisher();
    }

    CollisionInterface(ros::NodeHandle &nh, std::string link_name,
                       std::string scene_topic)
        : link_to_attach_to_(link_name) {
        nh_ = nh;
        planning_scene_topic_ = scene_topic;
        createPublisher();
    }

    CollisionInterface(ros::NodeHandle &nh,
                       const char *link_name = "left_gripper")
        : link_to_attach_to_(link_name) {
        nh_ = nh;
        createPublisher();

        // // for debugging we try to reload this from the rosparam server
        // // everytime we execute an add
        // nh_.param<std::string>("apc_object_attach_tf", link_to_attach_to_,
        //                        "left_endpoint");
    }

    ~CollisionInterface() {}

    // create and advertise the publisher to the plannig scene
    void createPublisher() {
        planning_scene_publisher_ = nh_.advertise<moveit_msgs::PlanningScene>(
            planning_scene_topic_.c_str(), 1);
        // "planning_scene", 1);
    }

    // setters and getters
    std::string getPlanningSceneTopic() { return planning_scene_topic_; }
    void setPlanningSceneTopic(std::string scene_topic) {
        planning_scene_topic_ = scene_topic;
    }

    int getNumSubscribers() {
        return planning_scene_publisher_.getNumSubscribers();
    }

    // bool apc_msgs::SelectGraspFromCandidates::Request &req,
    //     apc_msgs::SelectGraspFromCandidates::Response &res
    // bool attachObjectToRobot(const char object_id[]) {
    bool attachObjectModel(
        apc_msgs::AttachObjectCollisionModel::Request &req,
        apc_msgs::AttachObjectCollisionModel::Response &res) {
        // Create the name of the object out of the ID (integer) being sent
        std::string object_name = "object_";
        object_name.append(std::to_string(req.object_id.data));

        // req needs to contain an attachment frame
        // should be left_gripper or left_gripper_90
        std::string limb;
        if (!req.link_to_attach_to.data.empty()) {
            link_to_attach_to_ = req.link_to_attach_to.data;
            std::size_t found = link_to_attach_to_.find("left_");
            if (found!=std::string::npos){
              limb = "left_";
            } else {
              limb = "right_";
            }
        }

        ROS_INFO("[attachObjectModel] attaching object '%s' to the %s.",
                 object_name.c_str(), link_to_attach_to_.c_str());

        // create a collision object to be attached to the robot
        moveit_msgs::CollisionObject collision_object;
        collision_object.id = object_name;  // is a string for moveit
        // object reference frame (for pose, see below) is the same as then
        // attachment point
        collision_object.header.frame_id = link_to_attach_to_;
        // Note that attaching an object to the robot requires
        // the corresponding operation to be specified as an ADD operation
        collision_object.operation = moveit_msgs::CollisionObject::ADD;

        // TODO maybe get pose from somewhere? or can we assume it's the same
        // almost every time because of the way we suck?!
        fillCollisionModel(collision_object, req.radius.data);

        // TODO check if object is inthe world then we need to remove it from
        // there before adding it here
        // Note how we make sure that the diff message contains no other
        // attached objects or collisions objects by clearing those fields
        // first.
        /* Carry out the REMOVE + ATTACH operation */

        moveit_msgs::PlanningScene planning_scene_diff;
        // // need to set the diff, othrewise it delets all objects!
        planning_scene_diff.is_diff = true;
        // planning_scene_diff.robot_state.is_diff = true;

        moveit_msgs::AttachedCollisionObject attached_object;
        attached_object.link_name = link_to_attach_to_;
        attached_object.object = collision_object;
        // other links (apart from the attachment point) that the object
        // IS ALLOWED to collidie with
        attached_object.touch_links.push_back(limb + "gripper");
        attached_object.touch_links.push_back(limb + "gripper_90");
        attached_object.touch_links.push_back(limb + "gripper_base");
        attached_object.touch_links.push_back(limb + "hand");

        std::cout << "Collision object: " << attached_object << std::endl;

        planning_scene_diff.robot_state.attached_collision_objects.push_back(
            attached_object);

        planning_scene_publisher_.publish(planning_scene_diff);

        res.success.data = true;
        ROS_INFO("[attachObjectModel] object attached!");
        // TODO make use of return value?
        return true;
    }

    void fillCollisionModel(moveit_msgs::CollisionObject &obj,
                            float radius = 0.0f) {
        geometry_msgs::Pose pose;
        shape_msgs::SolidPrimitive primitive;

        // if not loading from database then
        if (0.0f < radius) {
            // TODO load that from a database?
            // loadCollisionGeometry(obj.id, pose, primitve);
            getDefaultGeometry(pose, primitive);
            primitive.type = primitive.SPHERE;
            primitive.dimensions.resize(1);
            primitive.dimensions[0] = radius;
            pose.position.x = radius + EPSILON;
        } else
            getDefaultGeometry(pose, primitive);

        obj.primitives.push_back(primitive);
        obj.primitive_poses.push_back(pose);
    }

    void loadCollisionGeometry(int id, geometry_msgs::Pose &pose,
                               shape_msgs::SolidPrimitive &primitive) {
        ROS_ERROR("loadCollisionGeometry() not implemented yet.");
    }

    void getDefaultGeometry(geometry_msgs::Pose &pose,
                            shape_msgs::SolidPrimitive &primitive) {
        // default geometry
        float cube_size = 0.2;  // 0.12;

        pose.orientation.w = 1.0;
        // pose.position.x = cube_size * 0.5;
        // for hand we need to put it along z further away
        pose.position.x = cube_size / 2.0 + EPSILON;

        // Collision Model
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[0] = cube_size;
        primitive.dimensions[1] = cube_size;
        primitive.dimensions[2] = cube_size;
    }

    // remove from world!
    // /* First, define the REMOVE object message*/
    // moveit_msgs::CollisionObject remove_object;
    // remove_object.id = object_id;
    // remove_object.header.frame_id = frame_id;
    // remove_object.operation = remove_object.REMOVE;

    // planning_scene.world.collision_objects.clear();
    // planning_scene.world.collision_objects.push_back(remove_object);

    // // Attach an object to the robot
    // // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    // // When the robot picks up an object from the environment, we need to
    // // "attach" the object to the robot so that any component dealing with
    // // the robot model knows to account for the attached object, e.g. for
    // // collision checking.
    //
    // // Attaching an object requires two operations
    // //  * Removing the original object from the environment
    // //  * Attaching the object to the robot
    //

    bool detachObjectModel(
        apc_msgs::DetachObjectCollisionModel::Request &req,
        apc_msgs::DetachObjectCollisionModel::Response &res) {
        // req needs to contain an attachment frame
        // should be left_gripper or left_gripper_90 or right_gripper or right_gripper_90
        if (!req.link_it_is_attached_to.data.empty()) {
            link_to_attach_to_ = req.link_it_is_attached_to.data;
        }
        ROS_INFO("Detaching the object from the %s.",
                 link_to_attach_to_.c_str());

        std::string object_id = "object_";
        object_id.append(std::to_string(req.object_id.data));

        // TODO sanity check of planning_scene_publisher,
        moveit_msgs::AttachedCollisionObject detach_object;
        detach_object.link_name = link_to_attach_to_;
        detach_object.object.header.frame_id = link_to_attach_to_;
        detach_object.object.id = object_id;
        detach_object.object.operation = moveit_msgs::CollisionObject::REMOVE;

        // Note how we make sure that the diff message contains no other
        // attached objects or collisions objects by clearing those fields
        // first.
        moveit_msgs::PlanningScene planning_scene;
        // TODO look as this?
        planning_scene.is_diff = true;
        planning_scene.robot_state.attached_collision_objects.clear();
        planning_scene.robot_state.attached_collision_objects.push_back(
            detach_object);

        planning_scene_publisher_.publish(planning_scene);
        ROS_INFO("[detachObjectModel] object detached!");

        // TODO make use of return value
        return true;
    }
    //
    // int detachObjectFromRobot(const char object_id[]) {
    // // // Detach an object from the robot
    // // // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    // // // Detaching an object from the robot requires two operations
    // // //  * Detaching the object from the robot
    // // //  * Re-introducing the object into the environment
    // //
    // //   /* First, define the DETACH object message*/
    // //   moveit_msgs::AttachedCollisionObject detach_object;
    // //   detach_object.object.id = "box";
    // //   detach_object.link_name = "r_wrist_roll_link";
    // //   detach_object.object.operation = detached_object.object.REMOVE;
    // //
    // // // Note how we make sure that the diff message contains no other
    // // // attached objects or collisions objects by clearing those fields
    // // // first.
    // //   /* Carry out the DETACH + ADD operation */
    // //   ROS_INFO("Detaching the object from the robot and returning it to
    // the world.");
    // //   planning_scene.robot_state.attached_collision_objects.clear();
    // //
    // planning_scene.robot_state.attached_collision_objects.push_back(detach_object);
    // //   planning_scene.world.collision_objects.clear();
    // //
    // planning_scene.world.collision_objects.push_back(attached_object.object);
    // //   planning_scene_diff_publisher.publish(planning_scene);
    //     ROS_INFO("[detachObjectFromRobot] Function not yet implemented!");
    //     return 0;
    // }

    //
    // // REMOVE THE OBJECT FROM THE COLLISION WORLD
    // // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    // // Removing the object from the collision world just requires
    // // using the remove object message defined earlier.
    // // Note, also how we make sure that the diff message contains no other
    // // attached objects or collisions objects by clearing those fields
    // // first.
    //   ROS_INFO("Removing the object from the world.");
    //   planning_scene.robot_state.attached_collision_objects.clear();
    //   planning_scene.world.collision_objects.clear();
    //   planning_scene.world.collision_objects.push_back(remove_object);
    //   planning_scene_diff_publisher.publish(planning_scene);
    // // END_TUTORIAL
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "apc_collision_object_service");
    ROS_INFO("Staring one-time adding of collision model for grasped object");

    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::NodeHandle node_hdl;

    ROS_INFO("getting planning scene...");

    std::string link_name, scene_topic;
    // node_hdl.param<std::string>("apc_object_attach_tf", link_name,
    //                             "left_gripper");
    node_hdl.param<std::string>("apc_planning_scene_topic", scene_topic,
                                "planning_scene");

    // TODO setup planning scene interfafce
    // TODO figure out if left or right from state machine probably
    // CollisionInterface object_collision_service(node_hdl, link_name,
    //                                             scene_topic);

    CollisionInterface object_collision_service(node_hdl, scene_topic);

    // ros::Subscriber sub_message = n.subscribe(topic.c_str(), 1000,
    // &NodeExample::messageCallback, node_example);

    // Advertise Services to be used (and implementd here)
    ros::ServiceServer srv_attach_ = node_hdl.advertiseService(
        "apc_grasping/attach_object_collision_model",
        &CollisionInterface::attachObjectModel, &object_collision_service);

    ros::ServiceServer srv_detach_ = node_hdl.advertiseService(
        "apc_grasping/detach_object_collision_model",
        &CollisionInterface::detachObjectModel, &object_collision_service);

    ROS_INFO(
        "CollisionService started! Ready to add object models to the planning "
        "scene ...");

    ros::Rate rate(50);
    while (ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }

    ros::shutdown();
    return 0;
}
