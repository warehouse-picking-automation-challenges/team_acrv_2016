#ifndef ROS_KINFU_H
#define ROS_KINFU_H

#define _CRT_SECURE_NO_DEPRECATE

///////////////////////////////////////////////////////////
// ROS

#include <eigen_conversions/eigen_msg.h>
#include <pcl_conversions/pcl_conversions.h>

#include <ros/spinner.h>
#include <std_msgs/Empty.h>
#include <std_srvs/Empty.h>
#include <apc_msgs/GetKinfuPointCloud.h>

#include <image_transport/camera_subscriber.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/fill_image.h>

#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>

//#include <pcl/ros/conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <cv_bridge/cv_bridge.h>

///////////////////////////////////////////////////////////////////
// Standard

#include <boost/filesystem.hpp>
#include <iostream>
#include <vector>
#include <string>

///////////////////////////////////////////////////////////////////
// PCL

#include <pcl/common/angles.h>
#include <pcl/common/time.h>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <pcl/exceptions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/image_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>

#include <pcl/gpu/containers/initialization.h>
#include <pcl/gpu/kinfu/internal.h>
#include <pcl/gpu/kinfu/kinfu.h>
#include <pcl/gpu/kinfu/marching_cubes.h>
#include <pcl/gpu/kinfu/raycaster.h>
#include <pcl/gpu/kinfu/tools/camera_pose.h>
#include <pcl/gpu/kinfu/tools/tsdf_volume.h>
#include <pcl/gpu/kinfu/tools/tsdf_volume.hpp>

/////////////////////////////////////////////////////////////////////
// OPENCV

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

/////////////////////////////////////////////////////////////////////
// LOCAL

#include "kinfu_tools.h"
#include "kinfu_viz_tools.h"
#include "ros_kinfu_publisher.h"

using namespace std;
using namespace pcl;
using namespace pcl::gpu;
using namespace Eigen;

// typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image,
// sensor_msgs::Image, sensor_msgs::CameraInfo> SyncPolicy;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
                                                        sensor_msgs::Image>
    SyncPolicy;
// typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image,
// sensor_msgs::Image> SyncPolicy;

typedef pcl::PointXYZRGB PointRGB;
typedef pcl::PointXYZI PointTSDF;
typedef pcl::PointCloud<PointRGB> PointCloudRGB;
typedef pcl::PointCloud<PointTSDF> PointCloudTSDF;

static void keyboard_callback(const pcl::visualization::KeyboardEvent &e,
                              void *cookie);

class ros_kinfu {
   public:
    enum { PCD_BIN = 1, PCD_ASCII = 2, PLY = 3, MESH_PLY = 7, MESH_VTK = 8 };

    ros_kinfu(boost::shared_ptr<CameraPoseProcessor> pose_processor =
                  boost::shared_ptr<CameraPoseProcessor>());
    ~ros_kinfu();

    void initCurrentFrameView();
    void initRegistration();
    void setDepthIntrinsics();
    void setDepthIntrinsics(std::vector<float> depth_intrinsics);

    void toggleColorIntegration();
    void enableTruncationScaling();
    void toggleIndependentCamera();
    void togglePublisher();

    void cameraInfoCallback(const sensor_msgs::CameraInfo &msg);
    bool changeCameraCallback(std_srvs::Empty::Request &req,
                                         std_srvs::Empty::Response &res);

    void resetCallback(const std_msgs::Empty & /*msg*/);
    void pauseCallback(const std_msgs::Empty & /*msg*/);

    void execute(const sensor_msgs::Image::ConstPtr rgb,
                 const sensor_msgs::Image::ConstPtr depth,
                 const sensor_msgs::CameraInfo::ConstPtr cameraInfo);
    void execute(const sensor_msgs::Image::ConstPtr rgb,
                 const sensor_msgs::Image::ConstPtr depth);
    void execute(const sensor_msgs::Image::ConstPtr &depth);
    void startMainLoop();

    void writeCloud(int format) const;
    void writeMesh(int format) const;
    void printHelp();

    void publishTSDFCloud();
    void publishCameraPose();
    bool getCloudCallback(apc_msgs::GetKinfuPointCloud::Request &req,
    apc_msgs::GetKinfuPointCloud::Response &res);

    void readImageRGB(const sensor_msgs::Image::ConstPtr msgImage,
                      cv::Mat &image);

    void reset_kf_world();

    /////////////////////////////////////////////////////////////////////
    // Kinfu Member Variables

    bool exit_, scan_, scan_mesh_, scan_volume_, independent_camera_,
        registration_, integrate_colors_, pcd_source_;

    bool use_hints_, update_kinect_world_;

    KinfuTracker kinfu_;
    KinfuTracker::DepthMap depth_device_;
    KinfuTracker::DepthMap generated_depth_;

    SceneCloudView scene_cloud_view_;
    ImageView image_view_;
    boost::shared_ptr<CurrentFrameCloudView> current_frame_cloud_view_;

    pcl::TSDFVolume<float, short> tsdf_volume_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr tsdf_cloud_ptr_;

    boost::mutex data_ready_mutex_;
    boost::condition_variable data_ready_cond_;

    std::vector<KinfuTracker::PixelRGB> source_image_data_;
    std::vector<unsigned short> source_depth_data_;

    int time_ms_, icp_;
    bool viz_;

    // Depth Intrinsic parameters
    double fx_, fy_, cx_, cy_;

    boost::shared_ptr<CameraPoseProcessor> pose_processor_;

    ///////////////////////////////////////////////////////////////////////////////////////////////////

    // ROS Member Variables
    bool subsriber_start_, publish_;

    std::string topicDepth, topicColor, topicCameraInfo, topicReset, topicPause;
    std::string camera_frame_id;
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::SubscriberFilter *colorImageSubscriberFilter,
        *depthImageSubscriberFilter;
    image_transport::Subscriber depthImageSubscriber;
    ros::Subscriber cameraInfoSubscriber;
    message_filters::Synchronizer<SyncPolicy> *sync;
    ros::Subscriber resetSubscriber;
    ros::Subscriber pauseSubscriber;
    ros::ServiceServer service;
    ros::ServiceServer service_cc;

    Affine3f reverse_initial_pose;
    tf::TransformListener tf_listener;

    ros_kinfu_publisher kinfu_publisher;
    std::clock_t start;

    bool intrins_unset = true;

    bool reset_command_ = true;
    bool pause_command_ = false;
};

#endif  // ROS_KINFU_H
