#include "ros_kinfu_publisher.h"




ros_kinfu_publisher::ros_kinfu_publisher(ros::NodeHandle nodehandle):nh_(nodehandle),it_(nodehandle)
{

    if (!nh_.getParam("tsdf_topic", tsdf_topic)) tsdf_topic = "/ros_kinfu/tsdf/points";
    if (!nh_.getParam("cloud_topic", cloud_topic)) cloud_topic = "/ros_kinfu/depth/points";
    if (!nh_.getParam("cloud_colour_topic", cloud_color_topic))cloud_color_topic = "/ros_kinfu/depth_registered/points";
    if (!nh_.getParam("depth_output_image_topic", depth_topic))depth_topic = "/ros_kinfu/depth/image";
    if (!nh_.getParam("camera_pose_topic", camera_pose_topic))camera_pose_topic = "/ros_kinfu/camera/pose";
    if (!nh_.getParam("kf_world_frame_id", kf_world_frame))kf_world_frame = "/kf_world";
    if (!nh_.getParam("kf_camera_frame_id", kf_camera_frame))kf_camera_frame = "/kf_camera";


    depthPublisher = it_.advertise(depth_topic, 10);
    tsdfPublisher = nh_.advertise<sensor_msgs::PointCloud2>(tsdf_topic, 10);
    cloudPublisher = nh_.advertise<sensor_msgs::PointCloud2>(cloud_topic, 10);
    cloudColorPublisher = nh_.advertise<sensor_msgs::PointCloud2>(cloud_color_topic, 10);
    cameraPosePublisher = nh_.advertise<geometry_msgs::PoseStamped>(camera_pose_topic, 10);

}

void ros_kinfu_publisher::updateKinectWorldTransform(tf::StampedTransform stamped_transform, Affine3f input_pose){

    Affine3d kinect_camera_pose(input_pose);
    Affine3d kinect_inverse_pose = kinect_camera_pose.inverse();

    tf::Transform kinect_inverse_transform;
    tf::transformEigenToTF(kinect_inverse_pose,kinect_inverse_transform);

    tf::StampedTransform kinect_world_transform(stamped_transform*kinect_inverse_transform,ros::Time::now(),"/world",kf_world_frame);

    kf_world_transform = kinect_world_transform;
//    kf_world_transform.stamp_ = ros::Time::now();
//    kinfu_tf_broadcaster.sendTransform(kf_world_transform);

}

void ros_kinfu_publisher::publishTSDFCloud(PointCloudTSDF::Ptr cloud_ptr){

    sensor_msgs::PointCloud2 msg;
    PointCloudTSDF::Ptr tsdf_output_ptr(new PointCloudTSDF);

    Affine3f scale;
    scale = Scaling(1/1000.f);

    pcl::transformPointCloud(*cloud_ptr,*tsdf_output_ptr,scale);
    pcl::toROSMsg(*tsdf_output_ptr,msg);
    msg.header.frame_id = kf_world_frame;
    msg.header.stamp = ros::Time::now();

    tsdfPublisher.publish(msg);

}

void ros_kinfu_publisher::publishCloud(pcl::PointCloud<PointXYZ>::Ptr cloud_ptr){

    sensor_msgs::PointCloud2 msg;

    pcl::toROSMsg(*cloud_ptr,msg);
    msg.header.frame_id = kf_world_frame;
    msg.header.stamp = ros::Time::now();

    cloudPublisher.publish(msg);

}

void ros_kinfu_publisher::publishColorCloud(pcl::PointCloud<PointXYZRGB>::Ptr cloud_ptr){

    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*cloud_ptr,msg);
    msg.header.frame_id = kf_world_frame;
    msg.header.stamp = ros::Time::now();

    cloudColorPublisher.publish(msg);

}


void ros_kinfu_publisher::publishCameraPose(Affine3f pose_in){

    geometry_msgs::PoseStamped msg;

    Affine3d pose(pose_in);

    tf::poseEigenToMsg(pose,msg.pose);

    msg.header.frame_id = kf_camera_frame;
    msg.header.stamp = ros::Time::now();

    cameraPosePublisher.publish(msg);
}

void ros_kinfu_publisher::publishTFfromPose(Affine3f pose_in)
{
    tf::Transform transform;
    Affine3d pose(pose_in);
    tf::transformEigenToTF(pose,transform);

    kinfu_tf_broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), kf_world_frame, kf_camera_frame));
}

void ros_kinfu_publisher::publishKfWorldTransform()
{
    kf_world_transform.stamp_ = ros::Time::now();
    kinfu_tf_broadcaster.sendTransform(kf_world_transform);
}

void ros_kinfu_publisher::publishDepth(KinfuTracker::DepthMap depth)
{
    int c;
    vector<unsigned short> data;
    sensor_msgs::Image depth_msg;

    depth.download(data, c);
    //sensor_msgs::fillImage(depth_msg,sensor_msgs::image_encodings::MONO16,depth.rows(), depth.cols(), depth.cols(), &data[0]);
    sensor_msgs::fillImage(depth_msg, sensor_msgs::image_encodings::TYPE_16UC1, depth.rows(), depth.cols(),depth.cols()*2, &data[0]);
    depth_msg.header.frame_id = kf_camera_frame;
    depthPublisher.publish(depth_msg);
}

