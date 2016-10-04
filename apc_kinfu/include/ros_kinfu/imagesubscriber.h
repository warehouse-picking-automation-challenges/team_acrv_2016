#ifndef IMAGESUBSCRIBER_H
#define IMAGESUBSCRIBER_H

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

typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo> SyncPolicy;

class ImageSubscriber
{
    image_transport::ImageTransport it_;
    image_transport::SubscriberFilter *subImageColor, *subImageDepth;
    message_filters::Subscriber<sensor_msgs::CameraInfo> *subCameraInfo;
    message_filters::Synchronizer<SyncPolicy> *sync;

    ros::NodeHandle & nh_;
    boost::mutex & mutex;

public:

    ImageSubscriber();
    ~ImageSubscriber();
    void callback();
};

#endif // IMAGESUBSCRIBER_H
