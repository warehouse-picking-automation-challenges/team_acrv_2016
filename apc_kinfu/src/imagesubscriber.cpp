//#include "parameters.h"


#include <image_transport/image_transport.h>
#include <image_transport/camera_subscriber.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/fill_image.h>
#include <std_msgs/Empty.h>
#include <ros/spinner.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

ImageSubscriber::ImageSubscriber(ros::NodeHandle &n_handle,boost::mutex &shared_mutex):nh_(n_handle),it_(n_handle),mutex(shared_mutex)
{
    std::string prefix_topic;
    nh_.param<std::string>(PARAM_NAME_PREFIX_TOPIC,prefix_topic,PARAM_DEFAULT_PREFIX_TOPIC);

    std::string depth_image_topic;
    nh_.param<std::string>(PARAM_NAME_DEPTH_IMAGE_TOPIC,depth_image_topic,prefix_topic + PARAM_DEFAULT_DEPTH_IMAGE_TOPIC);

    std::string camera_info_topic;
    nh_.param<std::string>(PARAM_NAME_CAMERA_INFO_TOPIC,camera_info_topic,prefix_topic + PARAM_DEFAULT_CAMERA_INFO_TOPIC);

    std::string image_topic;
    nh_.param<std::string>(PARAM_NAME_IMAGE_TOPIC,image_topic,prefix_topic + PARAM_NAME_IMAGE_TOPIC);

    subImageColor = new image_transport::SubscriberFilter(it_, image_topic, 3);
    subImageDepth = new image_transport::SubscriberFilter(it_, depth_image_topic, 3);
    subCameraInfo = new message_filters::Subscriber<sensor_msgs::CameraInfo>(nh_, camera_info_topic, 3);

    sync = new message_filters::Synchronizer<SyncPolicy>(SyncPolicy(3), *subImageColor, *subImageDepth, *subCameraInfo);
    sync->registerCallback(boost::bind(&ImageSubscriber::callback, this, _1, _2, _3));

}


~ImageSubscriber();

void readImage(const sensor_msgs::Image::ConstPtr msgImage, cv::Mat &image)
{
    cv_bridge::CvImageConstPtr pCvImage;
    pCvImage = cv_bridge::toCvShare(msgImage, msgImage->encoding);
    pCvImage->image.copyTo(image);
}

void callback(const sensor_msgs::Image::ConstPtr imageColor, const sensor_msgs::Image::ConstPtr imageDepth,
              const sensor_msgs::CameraInfo::ConstPtr cameraInfo)
{

}
