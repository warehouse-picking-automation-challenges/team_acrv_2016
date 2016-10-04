#include <apc_grasping/Item.h>

Item::Item() {}

// Item constructor
Item::Item(const float positionX, const float positionY, const float positionZ,
    const float orientationX, const float orientationY,
    const float orientationZ, std::string item_id,
    moveit_msgs::PlanningScene* planning_scene_ptr) {
    setPose(positionX, positionY, positionZ, orientationX, orientationY,
        orientationZ);
    _planning_scene_ptr = planning_scene_ptr;
    _item_id = item_id;
}

// Item constructor
Item::Item(const float positionX, const float positionY, const float positionZ,
    const float orientationX, const float orientationY,
    const float orientationZ, std::string item_id) {
    setPose(positionX, positionY, positionZ, orientationX, orientationY,
        orientationZ);
    _item_id = item_id;
}

// Item constructor with pose
Item::Item(geometry_msgs::Pose pose, std::string item_id,
    moveit_msgs::PlanningScene* planning_scene_ptr) {
    _planning_scene_ptr = planning_scene_ptr;
    _item_id = item_id;
    _pose = pose;

    // Create transform for item
    _tfItem.setOrigin(tf::Vector3(_pose.position.x, _pose.position.y,
        _pose.position.z));
    _tfItem.setRotation(tf::Quaternion(_pose.orientation.x, _pose.orientation.y,
        _pose.orientation.z, _pose.orientation.w));

    // Create transform to the goal pose of the items
    _tfItemGoal.setOrigin(tf::Vector3(-0.20, 0, 0));
    tf::Quaternion q;
    q.setRPY(M_PI, -(M_PI / 2), 0);
    _tfItemGoal.setRotation(q);
}

// Item constructor with pose
Item::Item(geometry_msgs::Pose pose, std::string item_id) {
    _item_id = item_id;
    _pose = pose;

    // Create transform for item
    _tfItem.setOrigin(tf::Vector3(_pose.position.x, _pose.position.y,
        _pose.position.z));
    _tfItem.setRotation(tf::Quaternion(_pose.orientation.x, _pose.orientation.y,
        _pose.orientation.z, _pose.orientation.w));

    // Create transform to the goal pose of the items
    _tfItemGoal.setOrigin(tf::Vector3(-0.20, 0, 0));
    tf::Quaternion q;
    q.setRPY(M_PI, -(M_PI / 2), 0);
    _tfItemGoal.setRotation(q);
}

// Add a rectangle collision object into the moveit planner
void Item::addCollisionItem() {
    // while(_planning_scene_publisher.getNumSubscribers() < 1)
    // {
    //   ros::WallDuration sleep_t(0.5);
    //   sleep_t.sleep();
    // }

    // Create collision object
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = "/torso";
    collision_object.id = _item_id;

    // Load mesh from STL file
    shapes::Mesh* mesh =
        shapes::createMeshFromResource("package://apc_objects/meshes/visual_mesh/" + _item_id + ".STL");
    ROS_INFO("Mesh loaded.");

    // Create mesh for collision object
    shape_msgs::Mesh item_mesh;
    shapes::ShapeMsg item_mesh_msg;
    shapes::constructMsgFromShape(mesh, item_mesh_msg);
    item_mesh = boost::get<shape_msgs::Mesh>(item_mesh_msg);

    // Update the mesh and pose of the collision object
    collision_object.meshes.push_back(item_mesh);
    collision_object.mesh_poses.push_back(_pose);
    collision_object.operation = collision_object.ADD;

    // Remove any duplicates of the item from the plannnig scene
    // moveit_msgs::CollisionObject remove_object;
    // remove_object.header.frame_id = "/torso";
    // remove_object.id = _item_id;
    // remove_object.operation = collision_object.REMOVE;

    // Remove duplicates and publish the collision object to the planning scene
    // moveit_msgs::PlanningScene planning_scene;
    // planning_scene.world.collision_objects.clear();
    // _planning_scene_ptr->world.collision_objects.push_back(remove_object);
    // _planning_scene_publisher.publish(*_planning_scene_ptr);
    // ROS_INFO("Removed Object from world.");

    // _planning_scene_ptr->is_diff = true;
    _planning_scene_ptr->world.collision_objects.push_back(collision_object);
    // collision_objects.push_back(collision_object);
    ROS_INFO("Added Object into world.");
    // planning_scene_interface.addCollisionObjects(collision_objects);
    // ROS_INFO("Added a rectangle into the world");
}

float Item::getPosX() const {
    return _positionX;
}

float Item::getPosY() const {
    return _positionY;
}

float Item::getPosZ() const {
    return _positionZ;
}

float Item::getOrienX() const {
    return _orientationX;
}

float Item::getOrienY() const {
    return _orientationY;
}

float Item::getOrienZ() const {
    return _orientationZ;
}

// Set pose of item
void Item::setPose(const float posX, const float posY, const float posZ,
    const float orienX, const float orienY, const float orienZ) {
    // Set the position and orientation of the items pose
    _positionX = posX;
    _positionY = posY;
    _positionZ = posZ;
    _orientationX = orienX * (M_PI / 180);
    _orientationY = orienY * (M_PI / 180);
    _orientationZ = orienZ * (M_PI / 180);

    // Set the pose of the item
    _pose.position.x = _positionX;
    _pose.position.y = _positionY;
    _pose.position.z = _positionZ;
    _pose.orientation = tf::createQuaternionMsgFromRollPitchYaw
    (_orientationX, _orientationY, _orientationZ);

    // Create transform for item
    _tfItem.setOrigin(tf::Vector3(_pose.position.x, _pose.position.y,
        _pose.position.z));
    _tfItem.setRotation(tf::Quaternion(_pose.orientation.x, _pose.orientation.y,
        _pose.orientation.z, _pose.orientation.w));
}

// Set pose of item
void Item::setPose(geometry_msgs::Pose pose) {
    _pose = pose;

    // Create transform for item
    _tfItem.setOrigin(tf::Vector3(_pose.position.x, _pose.position.y,
        _pose.position.z));
    _tfItem.setRotation(tf::Quaternion(_pose.orientation.x, _pose.orientation.y,
        _pose.orientation.z, _pose.orientation.w));
}

void Item::broadcastTf() {
    tf::StampedTransform stampedTfBox(_tfItem, ros::Time::now(),
        "base", _item_id);

    // tf::StampedTransform stampedTfBoxTOGoal;
    // tf::Transformer box2Goal;

    br.sendTransform(stampedTfBox);
    ROS_INFO_STREAM("BROADCASTING: " << _item_id);
}

// Item destructor
Item::~Item() {}
