/*
Copyright 2016 Australian Centre for Robotic Vision
*/

#include "tf/transform_datatypes.h"
#include <pcl/filters/passthrough.h>
#include <math.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <vector>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <apc_3d_vision/split_labelled_point_cloud.h>
#include <apc_3d_vision.hpp>
#include <eigen_conversions/eigen_msg.h>
#include "tf_conversions/tf_eigen.h"
#include <map>
#include <utility>
#include <vector>
#include <stdio.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <pcl/visualization/cloud_viewer.h>
#include <boost/thread/thread.hpp>
#include <apc_msgs/CropCloud.h>
// global value to listen to transforms
boost::shared_ptr<tf::TransformListener> tf_listener_ptr;
tf::Transform tftote;
Apc3dVision apc_vis;
ros::Publisher debug1;
ros::Publisher debug2;

// Set a global value to us PI in shorthand
#define PI 3.14159265
// values to help with Calibration
int value = 0;
int valueMax = 255;
bool CALIBRATE = false;
bool publishTF = false;
// value to set calibration window name
static const std::string Window = "CALIBRATE windows";
// The results from calibration
uint8_t Hmin = 23;
uint8_t Smin = 184;
uint8_t Vmin = 0;
uint8_t Hmax = 111;
uint8_t Smax = 255;
uint8_t Vmax = 255;

bool lookupTransform(const std::string &fromFrame, const std::string &toFrame,
                     tf::StampedTransform &foundTransform) {
  try {
    ros::Time now = ros::Time::now();
    ros::Time zero = ros::Time(0);
    if (!tf_listener_ptr)
      ROS_ERROR("asdfsdaf");

    bool tf_success = tf_listener_ptr->waitForTransform(
        fromFrame, toFrame, zero, ros::Duration(3.0));

    if (tf_success) {
      tf_listener_ptr->lookupTransform(fromFrame, toFrame, zero,
                                       foundTransform);
    } else {
      ROS_WARN("Could not lookup transform.");
    }
  } catch (tf::TransformException ex) {
    ROS_ERROR("TransformException: %s", ex.what());
    return false;
  } catch (...) {
    ROS_ERROR("Unknown exception.");
    return false;
  }
  return true;
}
// Callback functions to set the values of the calibration
void ChangeHmin(int, void *) { Hmin = uint8_t(value); }
void ChangeSmin(int, void *) { Smin = uint8_t(value); }
void ChangeVmin(int, void *) { Vmin = uint8_t(value); }
void ChangeHmax(int, void *) { Hmax = uint8_t(value); }
void ChangeSmax(int, void *) { Smax = uint8_t(value); }
void ChangeVmax(int, void *) { Vmax = uint8_t(value); }

bool split_labelled_point_cloud(apc_msgs::CropCloud::Request &req,
                                apc_msgs::CropCloud::Response &res) {
  // std::cout << "/* I apologise in advance for the spam */" << std::endl;
  // get the boolean parameters
  bool updateTF;
  bool segmentPointcloud_;
  try {
    updateTF = req.updateTF.data;
  } catch (...) {
    updateTF = false;
  }
  try {
    segmentPointcloud_ = req.segmentPointcloud.data;
  } catch (...) {
    segmentPointcloud_ = false;
  }

  // Get the pointcloud out of the Request
  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> req_points;
  req_points.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
  try {
    pcl::fromROSMsg(req.input_cloud, *req_points);
  } catch (...) {
    // std::cout << "There is now points in this message!" << std::endl;
    res.success.data = false;
    return false;
  }

  // std::cout << "/* Ic'm a big boy now, I can cuss when I want */" <<
  // std::endl;

  pcl::PointCloud<pcl::PointXYZRGB> croppedCloud;
  pcl::PointCloud<pcl::PointXYZRGB> croppedCloud_inlier;
  if (CALIBRATE) {
    cv::Mat Color_cropped = cv::Mat(480, 640, CV_8UC3, cv::Scalar(0, 0, 0));
    int i = 0;
    BOOST_FOREACH (pcl::PointXYZRGB point, *req_points) {
      i++;
      if (point.r == 0) {
        continue;
      }
      cv::Mat M = cv::Mat(1, 1, CV_8UC3, cv::Scalar(point.r, point.g, point.b));
      cv::Mat M_hsv;
      cvtColor(M, M_hsv, CV_RGB2HSV);
      if (((M_hsv.data[0] < Hmin) || (M_hsv.data[0] > Hmax)) &&
          ((M_hsv.data[1] > Smin) && (M_hsv.data[1] < Smax)) &&
          ((M_hsv.data[2] > Vmin) && (M_hsv.data[2] < Vmax))) {
        croppedCloud.push_back(point);
        Color_cropped.data[3 * i + 2] = point.r;
        Color_cropped.data[3 * i + 1] = point.g;
        Color_cropped.data[3 * i] = point.b;
      }
    }

  } else {

    BOOST_FOREACH (pcl::PointXYZRGB point, *req_points) {
      if (point.r == 0) {
        continue;
      }
      cv::Mat M = cv::Mat(1, 1, CV_8UC3, cv::Scalar(point.r, point.g, point.b));
      cv::Mat M_hsv;
      cvtColor(M, M_hsv, CV_RGB2HSV);

      if (((M_hsv.data[0] < Hmin) || (M_hsv.data[0] > Hmax)) &&
          ((M_hsv.data[1] > Smin) && (M_hsv.data[1] < Smax)) &&
          ((M_hsv.data[2] > Vmin) && (M_hsv.data[2] < Vmax))) {
        croppedCloud.push_back(point);
      } else {
        croppedCloud_inlier.push_back(point);
      }
    }
  }

  if (segmentPointcloud_) {
    sensor_msgs::PointCloud2 cloud_inBox;
    try {
      pcl::toROSMsg(croppedCloud_inlier, cloud_inBox);
      cloud_inBox.header.stamp = ros::Time::now();
      cloud_inBox.header.frame_id = req.input_cloud.header.frame_id;
      res.segmented_cloud = cloud_inBox;
    } // save subtracted pointcloud
    catch (...) {
      std::cout << "You filtered everything and there's nothing in this cloud!"
                << std::endl;
      res.success.data = false;
      return false;
    }
  }
  if (updateTF) {
    // set up the eigen transform from the camera optical frame to to the torso
    Eigen::Affine3d camera_optical_to_torsoTL;
    tf::StampedTransform camera_optical_to_torso;
    lookupTransform("torso", "kf_world", camera_optical_to_torso);
    tf::transformTFToEigen(camera_optical_to_torso, camera_optical_to_torsoTL);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr croppedCloudPtr;
    croppedCloudPtr = boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>>(
        new pcl::PointCloud<pcl::PointXYZRGB>(croppedCloud));

    pcl::PointCloud<pcl::PointXYZ> guessTranslated_cropCloud;
    // set up values to help find the height of the tote from the torso
    float max = -100;
    std::vector<float> depths;
    int i = 0;
    // Code to find the tote hiehgt
    BOOST_FOREACH (pcl::PointXYZRGB point, *croppedCloudPtr) {
      if (point.x == 0) {
        continue;
      }
      // std::cout << point << std::endl;
      Eigen::Vector3d pointVector = Eigen::Vector3d(point.x, point.y, point.z);
      Eigen::Vector3d transformedPointVector =
          camera_optical_to_torsoTL * pointVector;
      pcl::PointXYZ point2;
      point2.x = transformedPointVector[0];
      point2.y = transformedPointVector[1];
      i++;
      if (0 == (i % 4)) {
        // std::cout << "/* message */" << std::endl;
        // printf("%f\n", );
        depths.push_back(transformedPointVector[2]);
      }
      point2.z = transformedPointVector[2];
      guessTranslated_cropCloud.push_back(point2);
      // std::cout <<"before :\t" <<pointVector << std::endl;
      // std::cout <<"Transformed:\t" <<transformedPointVector << std::endl;
    }
    // std::cout << depths << std::endl;
    std::sort(depths.begin(), depths.end());
    // max is tote height
    max = depths[depths.size() - 50];
    max = -.56;

    pcl::PointCloud<pcl::PointXYZ>::Ptr boxPtr(
        new pcl::PointCloud<pcl::PointXYZ>);

    int size_width = 356; // mm
    int size_height = 588; // mm
    int inner_width = 340; // mm
    int inner_height = 516; // mm
    int diff_width = (size_width - inner_width) / 2;
    int diff_height = (size_height - inner_height) / 2;
    // set up a 2d mask in the 3d plane to do ICP on the guessed plane
    for (int i = 0; i < size_height; i++) {
      for (int j = 0; j < size_width; j++) {
        pcl::PointXYZ new_point;
        if (i < diff_height) {
          new_point.x = (float)(i - size_height / 2) / 1000;
          new_point.y = (float)(j - size_width / 2) / 1000;
        } else if (((i > diff_height) && (i < (size_height - diff_height))) &&
                       (j < diff_width) ||
                   (j > (size_width - diff_width))) {
          new_point.x = (float)(i - size_height / 2) / 1000;
          new_point.y = (float)(j - size_width / 2) / 1000;
        } else if ((i > (size_height - diff_height))) {
          new_point.x = (float)(i - size_height / 2) / 1000;
          new_point.y = (float)(j - size_width / 2) / 1000;
        } else {
          continue;
        }
        new_point.z = 0;
        boxPtr->points.push_back(new_point);
      }
    }
    // Perform a passthrough filter from the top of the box to 5cm below.
    pcl::PointCloud<pcl::PointXYZ>::Ptr TranslatedPtr(
        new pcl::PointCloud<pcl::PointXYZ>(guessTranslated_cropCloud));
    std::cout << TranslatedPtr->size() << std::endl;
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(TranslatedPtr);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(max - 0.05, max + 0.02);
    pass.filter(*TranslatedPtr);
    std::cout << TranslatedPtr->size() << std::endl;

    // Flatten the point cloud to max height
    pcl::PointCloud<pcl::PointXYZ>::iterator itr = (*TranslatedPtr).begin();
    for (; itr != (*TranslatedPtr).end(); ++itr) {
      itr->z = max;
    }

    // retrieve the estimated position of the cloud wrt to the torso
    pcl::MomentOfInertiaEstimation<pcl::PointXYZ> feature_extractor;
    feature_extractor.setInputCloud(TranslatedPtr);
    feature_extractor.compute();
    // retrieve the moment of inertia and eccentricity
    std::vector<float> moment_of_inertia;
    std::vector<float> eccentricity;
    pcl::PointXYZ min_point_AABB;
    pcl::PointXYZ max_point_AABB;
    pcl::PointXYZ min_point_OBB;
    pcl::PointXYZ max_point_OBB;
    pcl::PointXYZ position_OBB;
    Eigen::Matrix3f rotational_matrix_OBB;
    float major_value, middle_value, minor_value;
    Eigen::Vector3f major_vector, middle_vector, minor_vector;
    Eigen::Vector3f mass_center;
    // std::cout << "/* message */" << std::endl;
    feature_extractor.getMomentOfInertia(moment_of_inertia);
    feature_extractor.getEccentricity(eccentricity);
    feature_extractor.getAABB(min_point_AABB, max_point_AABB);
    feature_extractor.getOBB(min_point_OBB, max_point_OBB, position_OBB,
                             rotational_matrix_OBB);
    feature_extractor.getEigenValues(major_value, middle_value, minor_value);
    feature_extractor.getEigenVectors(major_vector, middle_vector,
                                      minor_vector);
    feature_extractor.getMassCenter(mass_center);

    Eigen::Vector3f position(position_OBB.x, position_OBB.y, position_OBB.z);
    Eigen::Quaternionf quat(rotational_matrix_OBB);
    pcl::PointXYZ center(mass_center(0), mass_center(1), mass_center(2));
    pcl::PointXYZ x_axis(major_vector(0) + mass_center(0),
                         major_vector(1) + mass_center(1),
                         major_vector(2) + mass_center(2));
    pcl::PointXYZ y_axis(middle_vector(0) + mass_center(0),
                         middle_vector(1) + mass_center(1),
                         middle_vector(2) + mass_center(2));
    pcl::PointXYZ z_axis(minor_vector(0) + mass_center(0),
                         minor_vector(1) + mass_center(1),
                         minor_vector(2) + mass_center(2));
    // k matrix corresponding to the projected tote on the z=0 plane
    Eigen::Vector3d k = Eigen::Vector3d(mass_center(0), mass_center(1), 0);
    // calculate the angle from the x axis for the rotation ()
    float result = atan2(major_vector[1], major_vector[0]); // radians
    // the code below sets the position and rotation of the boxPtr to be
    // aligned with the guessed position of the tote wrt the torso
    // (also projected onto the z=0 plane)
    // translate and rotate the pointcloud with the the above values
    Eigen::Affine3f transform_b = Eigen::Affine3f::Identity();
    transform_b.translation() << mass_center(0), mass_center(1),
        k.matrix()(2, 0);
    Eigen::Affine3f initial_transform =
        transform_b.rotate(Eigen::AngleAxisf(result, Eigen::Vector3f::UnitZ()));
    pcl::transformPointCloud(*boxPtr, *boxPtr, initial_transform);

    // Publish the boxptr into the debug1
    sensor_msgs::PointCloud2 temp_msg;

    pcl::toROSMsg(*boxPtr, temp_msg);
    temp_msg.header.stamp = ros::Time::now();
    temp_msg.header.frame_id = "torso";
    debug1.publish(temp_msg);

    // perform the ICP from the project guessed z plane to the actual position
    Apc3dVision::icp_params_t icp_params(boxPtr, TranslatedPtr);
    icp_params.output_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    icp_params.input_cloud_leaf_size = 0.01;
    icp_params.is_downsample_input_cloud = true;
    icp_params.target_cloud_leaf_size = 0.01;
    icp_params.is_downsample_target_cloud = true;
    icp_params.verbose = false;
    apc_vis.align_icp_tote(&icp_params);

    pcl::transformPointCloud(*boxPtr, *boxPtr, icp_params.transform);

    // Publish the boxptr into the debug1
    pcl::toROSMsg(*boxPtr, temp_msg);
    temp_msg.header.stamp = ros::Time::now();
    temp_msg.header.frame_id = "torso";
    debug2.publish(temp_msg);

    // Now that we have the icp tranform, find the actual
    // transform of the box wrt to the
    Eigen::Affine3f transform_c = Eigen::Affine3f::Identity();
    Eigen::Affine3f transform_d = Eigen::Affine3f::Identity();
    transform_c.translation() << -(float)size_height / 2000.0, 0, 0;
    transform_d.translation() << 0, -(float)((size_width) / 2000.0), 0;
    Eigen::Affine3f e =
        transform_b * icp_params.transform * transform_c * transform_d;
    // Eigen::Affine3f e = transform_b*icp_params.transform;

    tftote.setOrigin(
        tf::Vector3(e.matrix()(0, 3), e.matrix()(1, 3), e.matrix()(2, 3)));

    tftote.setBasis(
        tf::Matrix3x3(e.matrix()(0, 0), e.matrix()(0, 1), e.matrix()(0, 2),
                      e.matrix()(1, 0), e.matrix()(1, 1), e.matrix()(1, 2),
                      e.matrix()(2, 0), e.matrix()(2, 1), e.matrix()(2, 2)));
    publishTF = true;
  }
  res.success.data = true;
  return true;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "cropTote");
  ros::NodeHandle nh_("~");
  // transform broadcaster to broadcast the recieved TF
  tf::TransformBroadcaster br;
  // Advertise the service to crop the cloud
  ros::ServiceServer service = nh_.advertiseService(
      "/apc_3d_vision/Crop_Tote_cloud", &split_labelled_point_cloud);
  tf_listener_ptr.reset(new tf::TransformListener());

  debug1 = nh_.advertise<sensor_msgs::PointCloud2>("/debug1", 1);
  ;
  debug2 = nh_.advertise<sensor_msgs::PointCloud2>("/debug2", 1);
  ;

  ros::AsyncSpinner spinner(2);
  spinner.start();
  if (CALIBRATE) {
    cv::namedWindow(Window);
    // char TrackbarName[50];
    // sprintf(TrackbarName, "Hmin %d", valueMax);
    cv::createTrackbar("H>Hmin", Window, &value, valueMax, ChangeHmin);

    // char TrackbarName2[50];
    // sprintf(TrackbarName2, "Smin %d", valueMax);
    cv::createTrackbar("S>Smin", Window, &value, valueMax, ChangeSmin);
    // char TrackbarName3[50];
    // sprintf(TrackbarName3, "Vmin %d", valueMax);
    cv::createTrackbar("V>Vmin", Window, &value, valueMax, ChangeVmin);

    // char TrackbarName4[50];
    // sprintf(TrackbarName4, "Hmax %d", valueMax);
    cv::createTrackbar("Hmax<H", Window, &value, valueMax, ChangeHmax);
    // char TrackbarName5[50];
    // sprintf(TrackbarName5, "Smax %d", valueMax);
    cv::createTrackbar("Smax<S", Window, &value, valueMax, ChangeSmax);
    // char TrackbarName6[50];
    // sprintf(TrackbarName6, "Vmax %d", valueMax);
    cv::createTrackbar("Vmax<V", Window, &value, valueMax, ChangeVmax);
  }
  while (ros::ok()) {
    if (publishTF)
      br.sendTransform(tf::StampedTransform(tftote, ros::Time::now(), "torso",
                                            "totePosition"));
  }

  return 0;
}
