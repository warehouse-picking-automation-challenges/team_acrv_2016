#ifndef ROS_KINFU_PUBLISHER_H
#define ROS_KINFU_PUBLISHER_H

//#include "ros_kinfu.h"

#include <image_transport/image_transport.h>
#include <eigen_conversions/eigen_msg.h>
#include <pcl_conversions/pcl_conversions.h>

#include <sensor_msgs/fill_image.h>
#include <sensor_msgs/Image.h>

#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>

////////////////////////////////////////////////////////
//PCL

#include <pcl/point_cloud.h>
#include <pcl/gpu/kinfu/kinfu.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>

using namespace Eigen;
using namespace std;
using namespace pcl;
using namespace pcl::gpu;
using namespace Eigen;

typedef pcl::PointXYZRGB PointRGB;
typedef pcl::PointXYZI PointTSDF;
typedef pcl::PointCloud<PointRGB> PointCloudRGB;
typedef pcl::PointCloud<PointTSDF> PointCloudTSDF;

class ros_kinfu_publisher
{
public:
    ros_kinfu_publisher(ros::NodeHandle nh_);
    void updateKinectWorldTransform(tf::StampedTransform odom_camera_transform, Affine3f input_pose);

    void publishTSDFCloud(PointCloudTSDF::Ptr cloud_ptr);
    void publishCameraPose(Affine3f pose);
    void publishTFfromPose(Affine3f pose);
    void publishDepth(KinfuTracker::DepthMap);
    void publishCloud(pcl::PointCloud<PointXYZ>::Ptr cloud_ptr);
    void publishColorCloud(pcl::PointCloud<PointXYZRGB>::Ptr cloud_ptr);
    void publishKfWorldTransform();
    //Camera Pose Transform
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    tf::TransformBroadcaster kinfu_tf_broadcaster;
    tf::StampedTransform kf_camera_transform, kf_world_transform;

    ros::Publisher cloudColorPublisher, cloudPublisher, tsdfPublisher, cameraPosePublisher;
    image_transport::Publisher depthPublisher;

    std::string tsdf_topic, camera_pose_topic, depth_topic, cloud_topic, cloud_color_topic;
    std::string kf_world_frame, kf_camera_frame;


};

#endif // ROS_KINFU_PUBLISHER_H
