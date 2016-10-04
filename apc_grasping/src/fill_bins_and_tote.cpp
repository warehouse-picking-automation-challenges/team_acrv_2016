/*
Copyright 2016 Australian Centre for Robotic Vision
*/

#include <geometry_msgs/Pose.h>
#include <ros/ros.h>

#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/PlanningScene.h>

#include <apc_msgs/FillUnfillBinsCollisionModel.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>

#include <xmlrpc.h>

#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

std::vector<std::string> BINS = {
    "bin_A", "bin_B", "bin_C", "bin_D", "bin_E", "bin_F",
    "bin_G", "bin_H", "bin_I", "bin_J", "bin_K", "bin_L",
};

double SHELF_LIP_THICKNESS = 0.0;

tf::Transform tote_tf;
bool publish_tf = false;
bool tf_published = false;
tf::TransformBroadcaster *br;

class CollisionInterface {
   private:
    std::string planning_scene_topic_;
    XmlRpc::XmlRpcValue shelf_layout_;
    XmlRpc::XmlRpcValue tote_information_;

    bool tote_on_;
    bool pillar_on_;
    ros::Publisher planning_scene_publisher_;
    ros::NodeHandle nh_;
    std::string current_bin;

   public:
    // Constuctor
    CollisionInterface() {
        ROS_INFO("[Constructor] we need a node handle at least...");
        exit(1);
    }

    CollisionInterface(ros::NodeHandle &nh, std::string scene_topic) {
        nh_ = nh;
        planning_scene_topic_ = scene_topic;
        init();
    }

    CollisionInterface(ros::NodeHandle &nh) {
        nh_ = nh;
        init();
    }

    ~CollisionInterface() {}

    // create and advertise the publisher to the planning scene
    void createPublisher() {
        planning_scene_publisher_ = nh_.advertise<moveit_msgs::PlanningScene>(
            planning_scene_topic_.c_str(), 1);
    }

    // get shelf layout from parameter server
    void getShelfLayout() { nh_.getParam("shelf_layout", shelf_layout_); }

    // get tote information from parameter server
    void getToteInformation() {
        nh_.getParam("tote_information", tote_information_);
    }

    // initialise various bits and pieces
    void init() {
        tote_on_ = false;
        pillar_on_ = false;
        createPublisher();
        getShelfLayout();
        getToteInformation();
    }

    // setters and getters
    std::string getPlanningSceneTopic() { return planning_scene_topic_; }

    void setPlanningSceneTopic(std::string scene_topic) {
        planning_scene_topic_ = scene_topic;
    }

    int getNumSubscribers() {
        return planning_scene_publisher_.getNumSubscribers();
    }

    bool toggleLipFillbox(std_srvs::SetBool::Request &req,
                          std_srvs::SetBool::Response &res) {
        getShelfLayout();
        std::string bin_name = current_bin;
        moveit_msgs::PlanningScene planning_scene_diff;

        if (req.data) {
            moveit_msgs::CollisionObject collision_object;

            collision_object.operation = moveit_msgs::CollisionObject::ADD;

            shape_msgs::SolidPrimitive primitive;
            primitive.type = primitive.BOX;

            geometry_msgs::Pose pose;

            planning_scene_diff.is_diff = true;

            planning_scene_diff.world.collision_objects.clear();

            double side_h = 0.08;
            double side_w = shelf_layout_[bin_name]["bin_width"];
            double side_d = 0.05;
            collision_object.id = bin_name + "_lip_fillbox";
            collision_object.header.frame_id = bin_name;
            primitive.dimensions = {side_h, side_w, side_d};
            collision_object.primitives.clear();
            collision_object.primitives.push_back(primitive);
            collision_object.primitive_poses.clear();
            setGeometry(pose, side_h, -1.0 * side_w, -1.0 * side_d);
            double bin_height = shelf_layout_[bin_name]["bin_height"];
            pose.position.x += bin_height - 0.04;
            ROS_INFO_STREAM(pose);
            collision_object.primitive_poses.push_back(pose);
            ROS_INFO_STREAM(collision_object);
            planning_scene_diff.world.collision_objects.push_back(
                collision_object);
        } else {
            moveit_msgs::CollisionObject detach_object;
            detach_object.operation = moveit_msgs::CollisionObject::REMOVE;
            moveit_msgs::PlanningScene planning_scene;
            planning_scene_diff.is_diff = true;
            planning_scene_diff.world.collision_objects.clear();

            detach_object.id = current_bin + "_lip_fillbox";

            ROS_INFO_STREAM(detach_object.id);
            detach_object.header.frame_id = current_bin;
            ROS_INFO_STREAM(detach_object);
            planning_scene_diff.world.collision_objects.push_back(
                detach_object);
        }

        try {
            planning_scene_publisher_.publish(planning_scene_diff);
            res.success = true;
            return true;
            // if any errors, return unsuccessful, should probably handle this
            // better
        } catch (...) {
            res.success = false;
            return false;
        }
    }

    bool fillBins(apc_msgs::FillUnfillBinsCollisionModel::Request &req,
                  apc_msgs::FillUnfillBinsCollisionModel::Response &res) {
        getShelfLayout();
        // in case unfill not previously called, remove box in current bin
        std_srvs::Trigger unfillSrvMsg;
        if (!unfillBins(unfillSrvMsg.request, unfillSrvMsg.response)) {
            ROS_INFO_STREAM("Unable to remove previous collision objects");
            res.success.data = false;
            return true;
        }

        if (req.bin_id.data.empty()) {
            ROS_INFO_STREAM("No bin_id provided!");
            res.success.data = false;
            return true;
        }
        std::string bin_name = req.bin_id.data;
        current_bin = bin_name;

        // create a collision object to be used to fill bins
        moveit_msgs::CollisionObject collision_object;
        collision_object.operation = moveit_msgs::CollisionObject::ADD;
        moveit_msgs::PlanningScene planning_scene_diff;
        geometry_msgs::Pose pose;
        planning_scene_diff.is_diff = true;
        planning_scene_diff.world.collision_objects.clear();
        double side_h = 0.04;
        double side_w = shelf_layout_[bin_name]["bin_width"];
        double side_d = 0.5;
        collision_object.id = bin_name + "_fillbox";
        collision_object.header.frame_id = bin_name;
        shape_msgs::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions = {side_h, side_w, side_d};
        collision_object.primitives.push_back(primitive);
        collision_object.primitive_poses.clear();
        setGeometry(pose, -1.0 * side_h, -1.0 * side_w, side_d);
        pose.position.x += SHELF_LIP_THICKNESS;
        collision_object.primitive_poses.push_back(pose);
        planning_scene_diff.world.collision_objects.push_back(collision_object);

        // lip fillbox
        side_h = 0.08;
        side_w = shelf_layout_[bin_name]["bin_width"];
        side_d = 0.05;
        collision_object.id = bin_name + "_lip_fillbox";
        collision_object.header.frame_id = bin_name;
        primitive.dimensions = {side_h, side_w, side_d};
        collision_object.primitives.clear();
        collision_object.primitives.push_back(primitive);
        collision_object.primitive_poses.clear();
        setGeometry(pose, side_h, -1.0 * side_w, -1.0 * side_d);
        double bin_height = shelf_layout_[bin_name]["bin_height"];
        pose.position.x += bin_height - 0.04;
        ROS_INFO_STREAM(pose);
        collision_object.primitive_poses.push_back(pose);
        ROS_INFO_STREAM(collision_object);
        planning_scene_diff.world.collision_objects.push_back(collision_object);

        // generate collision objects within shelves
        for (int i = 0; i < 12; i++) {
            if (BINS[i] == bin_name) continue;
            collision_object.id = BINS[i] + "_fillbox";
            collision_object.header.frame_id = BINS[i];
            fillCollisionModel(collision_object, i);
            planning_scene_diff.world.collision_objects.push_back(
                collision_object);
        }

        // thicken left wall of shelf
        side_h = 1.1;
        side_w = 0.05;
        side_d = 0.5;
        collision_object.id = "left_fillbox";
        collision_object.header.frame_id = "/bin_A";
        collision_object.primitives[0].dimensions.clear();
        collision_object.primitives[0].dimensions = {side_h, side_w, side_d};
        collision_object.primitive_poses.clear();
        setGeometry(pose, side_h, side_w, side_d);
        collision_object.primitive_poses.push_back(pose);
        planning_scene_diff.world.collision_objects.push_back(collision_object);

        // thicken right wall of shelf
        setGeometry(pose, side_h, -1.0 * side_w, side_d);
        collision_object.id = "right_fillbox";
        collision_object.header.frame_id = "/bin_C";
        collision_object.primitive_poses.clear();
        double offset = shelf_layout_["bin_C"]["bin_width"];
        pose.position.y -= offset;
        collision_object.primitive_poses.push_back(pose);
        planning_scene_diff.world.collision_objects.push_back(collision_object);

        // thicken top of shelf
        side_h = 0.05;
        side_w = 0.8;
        side_d = 0.5;
        setGeometry(pose, -1.0 * side_h, -1.0 * side_w, side_d);
        collision_object.id = "top_fillbox";
        collision_object.header.frame_id = "/bin_A";
        collision_object.primitives[0].dimensions.clear();
        collision_object.primitives[0].dimensions = {side_h, side_w, side_d};
        collision_object.primitive_poses.clear();
        collision_object.primitive_poses.push_back(pose);
        planning_scene_diff.world.collision_objects.push_back(collision_object);

        // thicken botto lip
        setGeometry(pose, side_h, -1.0 * side_w, side_d);
        offset = shelf_layout_["bin_J"]["bin_height"];
        pose.position.x += offset;
        collision_object.id = "bottom_fillbox";
        collision_object.header.frame_id = "/bin_J";
        collision_object.primitives[0].dimensions.clear();
        collision_object.primitives[0].dimensions = {side_h, side_w, side_d};
        collision_object.primitive_poses.clear();
        collision_object.primitive_poses.push_back(pose);
        planning_scene_diff.world.collision_objects.push_back(collision_object);

        ROS_INFO("[fillBins] filling all bins except %s", bin_name.c_str());

        // publish scene
        try {
            planning_scene_publisher_.publish(planning_scene_diff);
            res.success.data = true;
        } catch (...) {
            res.success.data = false;
        }

        ROS_INFO("Bins filled!");

        return true;
    }

    void fillCollisionModel(moveit_msgs::CollisionObject &obj, int bin_id) {
        geometry_msgs::Pose pose;
        shape_msgs::SolidPrimitive primitive;
        primitive.type = primitive.BOX;

        double height = shelf_layout_[BINS[bin_id]]["bin_height"];
        double width = shelf_layout_[BINS[bin_id]]["bin_width"];
        double depth = 0.5;

        // use bin row heights and column widths
        primitive.dimensions = {height, width, depth};

        setGeometry(pose, height, -1.0 * width, depth);
        obj.primitives.clear();
        obj.primitive_poses.clear();
        obj.primitives.push_back(primitive);
        obj.primitive_poses.push_back(pose);
    }

    // add appropriate offset with respect to bin TFs
    void setGeometry(geometry_msgs::Pose &pose, double x, double y, double z) {
        pose.position.x = x;
        pose.position.x /= 2.0f;
        pose.position.x -= SHELF_LIP_THICKNESS;
        pose.position.y = y;
        pose.position.y /= 2.0f;
        pose.position.z = z;
        pose.position.z /= 2.0f;
    }

    void loadCollisionGeometry(int id, geometry_msgs::Pose &pose,
                               shape_msgs::SolidPrimitive &primitive) {
        ROS_ERROR("loadCollisionGeometry() not implemented yet.");
    }

    // remove all bin fills and wall thickeners
    bool unfillBins(std_srvs::Trigger::Request &req,
                    std_srvs::Trigger::Response &res) {
        getShelfLayout();
        moveit_msgs::CollisionObject detach_object;
        detach_object.operation = moveit_msgs::CollisionObject::REMOVE;
        moveit_msgs::PlanningScene planning_scene;
        planning_scene.is_diff = true;
        planning_scene.world.collision_objects.clear();

        // remove all bin fillers
        for (int i = 0; i < 12; i++) {
            detach_object.id = BINS[i] + "_fillbox";
            detach_object.header.frame_id = BINS[i];
            planning_scene.world.collision_objects.push_back(detach_object);
            detach_object.id = BINS[i] + "_lip_fillbox";
            detach_object.header.frame_id = BINS[i];
            planning_scene.world.collision_objects.push_back(detach_object);
        }

        // remove wall thickeners
        detach_object.id = "left_fillbox";
        detach_object.header.frame_id = "bin_A";
        planning_scene.world.collision_objects.push_back(detach_object);
        detach_object.id = "top_fillbox";
        planning_scene.world.collision_objects.push_back(detach_object);
        detach_object.id = "right_fillbox";
        detach_object.header.frame_id = "bin_C";
        planning_scene.world.collision_objects.push_back(detach_object);
        detach_object.id = "bottom_fillbox";
        detach_object.header.frame_id = "bin_J";
        planning_scene.world.collision_objects.push_back(detach_object);

        // publish object removals
        try {
            planning_scene_publisher_.publish(planning_scene);
            res.success = true;
            ROS_INFO_STREAM("Bin fill collision objects removed!");
        } catch (...) {
            res.success = false;
        }

        return true;
    }

    // add and remove the tote and tote pillar
    bool moveTote(apc_msgs::FillUnfillBinsCollisionModel::Request &req,
                  apc_msgs::FillUnfillBinsCollisionModel::Response &res) {

        // Get the pose of the tote with respect to torso from the parameter server
        XmlRpc::XmlRpcValue tote = tote_information_["tote"];
        geometry_msgs::Pose tote_pose;
        double pos_x = tote["position_x"];
        double box_x = tote["box_x"];
        double pos_y = tote["position_y"];
        double box_y = tote["box_y"];
        double pos_z = tote["position_z"];
        double box_z = tote["box_z"];
        tote_pose.position.x = pos_x;
        tote_pose.position.y = pos_y;
        tote_pose.position.z = pos_z;
        tf::Quaternion tote_orientation;
        tote_orientation.setRPY(((double)tote["orientation_roll"]),
                                ((double)tote["orientation_pitch"]),
                                ((double)tote["orientation_yaw"]));
        tf::quaternionTFToMsg(tote_orientation, tote_pose.orientation);
        tf::poseMsgToTF(tote_pose, tote_tf);
        geometry_msgs::Pose not_tote_pose;

        not_tote_pose.position.x = box_x / 2.0;
        not_tote_pose.position.y = box_y / 2.0;
        not_tote_pose.position.z = -box_z / 2.0;
        not_tote_pose.orientation.x = 0.0;
        not_tote_pose.orientation.y = 0.0;
        not_tote_pose.orientation.z = 0.0;
        not_tote_pose.orientation.w = 1.0;
        tf::Transform not_tote_tf;
        tf::poseMsgToTF(not_tote_pose, not_tote_tf);

        not_tote_tf = tote_tf * not_tote_tf;
        tf::poseTFToMsg(not_tote_tf,not_tote_pose);

        publish_tf = true;

        bool tote_on = false;
        bool pillar_on = false;

        moveit_msgs::CollisionObject obj;
        obj.header.frame_id = "/torso";  // Always publish with respect to torso
        tote_on = tote_on_;
        pillar_on = pillar_on_;
        if (req.bin_id.data == "tote_on") {
            ROS_INFO_STREAM("tote_on");
            tote_on = true;
        } else if (req.bin_id.data == "tote_off") {
            ROS_INFO_STREAM("tote_off");
            tote_on = false;
        } else if (req.bin_id.data == "pillar_on") {
            ROS_INFO_STREAM("pillar_on");
            pillar_on = true;
        } else if (req.bin_id.data == "pillar_off") {
            ROS_INFO_STREAM("pillar_off");
            pillar_on = false;
        } else if (req.bin_id.data == "all_on") {
            ROS_INFO_STREAM("all_on");
            tote_on = true;
            pillar_on = true;
        } else if (req.bin_id.data == "all_off") {
            ROS_INFO_STREAM("all_off");
            tote_on = false;
            pillar_on = false;
        }

        moveit_msgs::PlanningScene planning_scene;
        planning_scene.world.collision_objects.clear();
        planning_scene.is_diff = true;
        nh_.getParam("tote_information", tote_information_);

        // geometry_msgs::Pose pose;
        shape_msgs::SolidPrimitive primitive;
        primitive.type = primitive.BOX;

        if (tote_on) {
            obj.id = "tote";
            obj.operation = moveit_msgs::CollisionObject::ADD;
            // add tote
            // dimensions
            primitive.dimensions = {box_x, box_y,
                                    box_z};
            // // position

            // tote_pose.position.x = pos_x + box_x*sin((double)tote["orientation_yaw"]) / 2.0;
            // tote_pose.position.y = pos_y + box_y*sin((double)tote["orientation_yaw"]) / 2.0;
            // tote_pose.position.z = pos_z - box_z / 2.0;

            ROS_INFO_STREAM("Tote position: " << not_tote_pose);
            // add obj to scene
            obj.primitives.push_back(primitive);
            obj.primitive_poses.push_back(not_tote_pose);
            planning_scene.world.collision_objects.push_back(obj);
            ROS_INFO_STREAM("Tote added to Collision Scene!");

        } else {
            obj.id = "tote";
            obj.operation = moveit_msgs::CollisionObject::REMOVE;
            planning_scene.world.collision_objects.push_back(obj);
            ROS_INFO_STREAM("Tote removed from Collision Scene!");
        }
        obj.primitives.clear();
        obj.primitive_poses.clear();
        if (pillar_on) {
            // obj.id = "tote_pillar";
            // obj.operation = moveit_msgs::CollisionObject::ADD;
            // // add tote_pillar
            // XmlRpc::XmlRpcValue tote_pillar = tote_information_["tote_pillar"];
            // // dimensions
            // primitive.dimensions = {tote_pillar["box_x"], tote_pillar["box_y"],
            //                         tote_pillar["box_z"]};
            // // position
            // pose.position.x = -((double)tote_pillar["position_x"]) / 2.0;
            // pose.position.y = -((double)tote_pillar["position_y"]) / 2.0;
            // pose.position.z = ((double)tote_pillar["position_z"]) / 2.0 + ((double)tote["box_z"]) / 2.0;
            // // add obj to scene
            // obj.id = "tote_pillar";
            // obj.primitives.push_back(primitive);
            // obj.primitive_poses.push_back(pose);
            // planning_scene.world.collision_objects.push_back(obj);
            // ROS_INFO_STREAM("Tote pillar added to Collision Scene!");

            // if tote is in place, remove from scene
        } else {
            // obj.id = "tote_pillar";
            // obj.operation = moveit_msgs::CollisionObject::REMOVE;
            // planning_scene.world.collision_objects.push_back(obj);
            // ROS_INFO_STREAM("Tote pillar removed from Collision Scene!");
        }

        // publish scene
        try {

            planning_scene_publisher_.publish(planning_scene);
            res.success.data = true;
            tote_on_ = tote_on;
            pillar_on_ = pillar_on;

            // if any errors, return unsuccessful, should probably handle this
            // better
        } catch (...) {
            res.success.data = false;
            return false;
        }

        return true;
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "fill_bins_and_tote");
    ROS_INFO("Starting bin filling and tote placement node!");

    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::NodeHandle node_hdl;

    br = new tf::TransformBroadcaster();

    std::string scene_topic;
    node_hdl.param<std::string>("apc_planning_scene_topic", scene_topic,
                                "planning_scene");

    CollisionInterface object_collision_service(node_hdl, scene_topic);

    // Advertise Services to be used (and implementd here)
    ros::ServiceServer srv_attach_ = node_hdl.advertiseService(
        "apc_grasping/fill_bins", &CollisionInterface::fillBins,
        &object_collision_service);

    ros::ServiceServer srv_detach_ = node_hdl.advertiseService(
        "apc_grasping/unfill_bins", &CollisionInterface::unfillBins,
        &object_collision_service);

    ros::ServiceServer srv_tote_inplace_ = node_hdl.advertiseService(
        "apc_grasping/move_tote", &CollisionInterface::moveTote,
        &object_collision_service);

    ros::ServiceServer srv_toggle_lipfillbox_ = node_hdl.advertiseService(
        "apc_grasping/toggle_lip", &CollisionInterface::toggleLipFillbox,
        &object_collision_service);

    ROS_INFO(
        "fill_bins_and_tote node started! ready to add toggle collision "
        "objects to the scene...");

    while (ros::ok()) {
        if (publish_tf){

            br->sendTransform(tf::StampedTransform(tote_tf, ros::Time::now(),
                                              "torso", "tote"));
            tf_published = true;
        }
        ros::spinOnce();
    }

    ros::shutdown();
    return 0;
}
