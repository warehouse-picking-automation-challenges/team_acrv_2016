#include <apc_msgs/DoSegmentation.h>
#include <apc_msgs/FillUnfillBinsCollisionModel.h>
#include <camera_info_manager/camera_info_manager.h>
#include <cv_bridge/cv_bridge.h>
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/TransformStamped.h>
#include <image_transport/image_transport.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/box_clipper3D.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/plane_clipper3D.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;

ros::Publisher point_pub;
image_transport::CameraPublisher depth_pub;
ros::Subscriber pointSub_;
ros::Subscriber imageSub_;
ros::Subscriber camera_info_sub;
image_transport::Publisher image_pub_;
boost::shared_ptr<tf::TransformListener> tf_listener_ptr;
XmlRpc::XmlRpcValue shelf_layout_;
bool publish_ = false;

std::string current_bin;
float bin_height;
float bin_width;
int image_width;
int image_height;

static const std::string OPENCV_WINDOW = "Cropped Image Window";
static const std::string OPENCV_WINDOW_depth = "depth window";
std::string ref_tf;

using namespace cv;
using namespace std;

bool DEBUG = false;

bool CALIBRATE = false;

// The following values are for the trackbar used
int value = 0;
int valueMax = 100;

cv::Point TL_points;
cv::Point TR_points;
cv::Point BL_points;
cv::Point BR_points;

cv::Point TL_points_d;
cv::Point TR_points_d;
cv::Point BL_points_d;
cv::Point BR_points_d;
Mat RGB_image;

float MAGIC_NUMBER = 0.02;

float c_width = 0.23;
float c_height = 0.23;
float c_widthOF = 0;
float c_heightOF = 0;

bool stop = true;
bool inloop_depth = false;
bool inloop_image = false;

float TL_Xd;
float TL_Yd;
float TL_Zd;
float TR_Xd;
float TR_Yd;
float TR_Zd;
float BL_Xd;
float BL_Yd;
float BL_Zd;
float BR_Xd;
float BR_Yd;
float BR_Zd;

void print_values() {
  std::cout << "variables: "
            << "[" << c_width << ", " << c_height << ", " << c_widthOF << ", "
            << c_heightOF << "]" << std::endl;
}
void ChangeWidth(int, void *) {
  c_width = (float)0.15f + 0.15f * (float)value / valueMax;
  print_values();
}

void ChangeHeight(int, void *) {
  c_height = (float)0.15f + 0.15f * (float)value / valueMax;
  print_values();
}
void ChangeWidthOffset(int, void *) {
  c_widthOF = -0.1f + 0.2f * (float)value / valueMax;
  print_values();
}
void ChangeHeightOffset(int, void *) {
  c_heightOF = -0.1f + 0.2f * (float)value / valueMax;
  print_values();
}

bool lookupTransform(const std::string &fromFrame, const std::string &toFrame,
                     tf::StampedTransform &foundTransform);

void getCurrentBin() {
  ros::NodeHandle nh;
  nh.getParam("/current_bin", current_bin);

  // values are needed as float
  double bin_height_temp = shelf_layout_[current_bin]["bin_height"];
  double bin_width_temp = shelf_layout_[current_bin]["bin_width"];
  bin_height = (float)bin_height_temp;
  bin_width = (float)bin_width_temp;
}

void updatePoints() {
  getCurrentBin();
  ros::NodeHandle nh("~");

  std::vector<float> binDim;
  std::string param_str = "/shelf_based_image_cropper_node/";
  param_str.append(current_bin);
  nh.getParam(param_str, binDim);

  // Set up the new dimensions for the image frame
  // get the focal length of x
  // get the focal length of y
  Eigen::Affine3d camera_to_bin_eigenTL;
  tf::StampedTransform camera_to_bin_TL;

  lookupTransform("/camera_rgb_optical_frame", current_bin, camera_to_bin_TL);

  tf::transformTFToEigen(camera_to_bin_TL, camera_to_bin_eigenTL);

  // TL is normal--- top left
  // TR is translated in negative y direction--- top right
  // BL is translated 0.265 in positive x direction--- bottom left
  // BR is translated in positive x direction and in negative y direction---
  // bottom right
  Eigen::Affine3d tl, tr, bl, br;
  if (CALIBRATE) {
    tl = (Eigen::Translation3d(Eigen::Vector3d(c_heightOF, -c_widthOF, 0)));
    tr = (Eigen::Translation3d(
        Eigen::Vector3d(c_heightOF, -(c_widthOF + c_width), 0)));
    bl = (Eigen::Translation3d(
        Eigen::Vector3d(c_heightOF + c_height, -c_widthOF, 0)));
    br = (Eigen::Translation3d(
        Eigen::Vector3d(c_heightOF + c_height, -(c_widthOF + c_width), 0)));
  } else {
    tl = (Eigen::Translation3d(Eigen::Vector3d(binDim[3], -binDim[2], 0)));
    tr = (Eigen::Translation3d(
        Eigen::Vector3d(binDim[3], -(binDim[2] + binDim[0]), 0)));
    bl = (Eigen::Translation3d(
        Eigen::Vector3d(binDim[3] + binDim[1], -binDim[2], 0)));
    br = (Eigen::Translation3d(
        Eigen::Vector3d(binDim[3] + binDim[1], -(binDim[2] + binDim[0]), 0)));
  }

  Eigen::Matrix4d TL = (camera_to_bin_eigenTL * tl).matrix();
  Eigen::Matrix4d TR = (camera_to_bin_eigenTL * tr).matrix();
  Eigen::Matrix4d BL = (camera_to_bin_eigenTL * bl).matrix();
  Eigen::Matrix4d BR = (camera_to_bin_eigenTL * br).matrix();

  // color focal lengths
  // fx616.873
  // fy616.874
  // depth focal lengths
  // fx476.898
  // fy476.898
  // color principal points
  // ppx315.057
  // ppy240.877
  // depth principal points
  // ppx309.477
  // ppy246.055

  float fx_d = 476.003;
  float fy_d = 476.003;
  float ppx_d = 308.131;
  float ppy_d = 246.076;

  TL_Xd = TL(0, 3);
  TL_Yd = TL(1, 3);
  TL_Zd = TL(2, 3);
  TR_Xd = TR(0, 3);
  TR_Yd = TR(1, 3);
  TR_Zd = TR(2, 3);
  BL_Xd = BL(0, 3);
  BL_Yd = BL(1, 3);
  BL_Zd = BL(2, 3);
  BR_Xd = BR(0, 3);
  BR_Yd = BR(1, 3);
  BR_Zd = BR(2, 3);

  int TL_1d = ppy_d + (TL_Yd * fy_d / TL_Zd);
  int TL_2d = ppx_d + (TL_Xd * fx_d / TL_Zd);
  int TR_1d = ppy_d + (TR_Yd * fy_d / TR_Zd);
  int TR_2d = ppx_d + (TR_Xd * fx_d / TR_Zd);
  int BL_1d = ppy_d + (BL_Yd * fy_d / BL_Zd);
  int BL_2d = ppx_d + (BL_Xd * fx_d / BL_Zd);
  int BR_1d = ppy_d + (BR_Yd * fy_d / BR_Zd);
  int BR_2d = ppx_d + (BR_Xd * fx_d / BR_Zd);
  TL_points_d = cv::Point(TL_2d, TL_1d);
  TR_points_d = cv::Point(TR_2d, TR_1d);
  BL_points_d = cv::Point(BL_2d, BL_1d);
  BR_points_d = cv::Point(BR_2d, BR_1d);

  // this is aligned to the color frame so we'll use color focal lengths
  float fx_c = 615.14215;
  float fy_c = 615.14215;
  float ppx_c = 312.925;
  float ppy_c = 241.733;

  float TL_X = TL(0, 3);
  float TL_Y = TL(1, 3);
  float TL_Z = TL(2, 3);
  float TR_X = TR(0, 3);
  float TR_Y = TR(1, 3);
  float TR_Z = TR(2, 3);
  float BL_X = BL(0, 3);
  float BL_Y = BL(1, 3);
  float BL_Z = BL(2, 3);
  float BR_X = BR(0, 3);
  float BR_Y = BR(1, 3);
  float BR_Z = BR(2, 3);

  int TL_1 = ppy_c + (TL_Y * fy_c / TL_Z);
  int TL_2 = ppx_c + (TL_X * fx_c / TL_Z);
  int TR_1 = ppy_c + (TR_Y * fy_c / TR_Z);
  int TR_2 = ppx_c + (TR_X * fx_c / TR_Z);
  int BL_1 = ppy_c + (BL_Y * fy_c / BL_Z);
  int BL_2 = ppx_c + (BL_X * fx_c / BL_Z);
  int BR_1 = ppy_c + (BR_Y * fy_c / BR_Z);
  int BR_2 = ppx_c + (BR_X * fx_c / BR_Z);

  TL_points = cv::Point(TL_2, TL_1);
  TR_points = cv::Point(TR_2, TR_1);
  BL_points = cv::Point(BL_2, BL_1);
  BR_points = cv::Point(BR_2, BR_1);
}

bool changeBin(apc_msgs::FillUnfillBinsCollisionModel::Request &req,
               apc_msgs::FillUnfillBinsCollisionModel::Response &res) {
  current_bin = req.bin_id.data;
  ros::NodeHandle nh;
  nh.setParam("/current_bin", current_bin);
  ROS_INFO_STREAM("New splitting bin: " << current_bin);
  res.success.data = true;
  stop = false;
  return true;
}

void splitCloud(PointCloud::Ptr cloud_in, PointCloud::Ptr cloud_out) {
  Eigen::Affine3d eigen_tf;
  tf::StampedTransform transform;
  getCurrentBin();

  ROS_INFO_STREAM("Finding bin to ref_tf transform");
  lookupTransform(current_bin, ref_tf, transform);

  tf::transformTFToEigen(transform, eigen_tf);

  std::cout << eigen_tf.matrix() << std::endl;
  pcl::transformPointCloud(*cloud_in, *cloud_in, eigen_tf);

  pcl::PassThrough<PointT> pass;
  ROS_INFO_STREAM("Begin filtering");

  pass.setInputCloud(cloud_in);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(0.01, 0.4);
  pass.filter(*cloud_in);

  pass.setInputCloud(cloud_in);
  pass.setFilterFieldName("x");
  pass.setFilterLimits(0.02, bin_height - MAGIC_NUMBER);
  pass.filter(*cloud_in);

  pass.setInputCloud(cloud_in);
  pass.setFilterFieldName("y");
  pass.setFilterLimits(-bin_width + MAGIC_NUMBER, 0.0);
  pass.filter(*cloud_in);

  pcl::transformPointCloud(*cloud_in, *cloud_out, eigen_tf.inverse());
  ROS_INFO_STREAM("Filtered");
}

void splitCloudKinect(PointCloud::Ptr cloud_in, PointCloud::Ptr cloud_out) {

  pcl::PassThrough<PointT> pass;
  ROS_INFO_STREAM("Begin filtering");

  pass.setInputCloud(cloud_in);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(0.3, 1.0);
  pass.filter(*cloud_out);

  ROS_INFO_STREAM("Filtered");
}

void splitCloudTote(PointCloud::Ptr cloud_in, PointCloud::Ptr cloud_out) {
  Eigen::Affine3d eigen_tf;
  tf::StampedTransform transform;
  getCurrentBin();
  ROS_INFO_STREAM(ref_tf);
  lookupTransform("tote", ref_tf, transform);

  tf::transformTFToEigen(transform, eigen_tf);

  pcl::transformPointCloud(*cloud_in, *cloud_in, eigen_tf);

  pcl::PassThrough<PointT> pass;
  ROS_INFO_STREAM("Begin filtering");

  pass.setInputCloud(cloud_in);
  pass.setFilterFieldName("y");
  pass.setFilterLimits(0.01,
                       0.35); // values roughly correct, may need minus sign
  pass.filter(*cloud_in);

  if (cloud_in->empty()) {
    ROS_INFO_STREAM("Empty cloud x!");
  }

  pass.setInputCloud(cloud_in);
  pass.setFilterFieldName("x");
  pass.setFilterLimits(0.04,
                       0.56); // values roughly correct, may need minus sign
  pass.filter(*cloud_in);

  if (cloud_in->empty()) {
    ROS_INFO_STREAM("Empty cloud y!");
  }

  pass.setInputCloud(cloud_in);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(-0.19, 1.0); // values roughly correct, may need minus
                                    // sign, large positive value ensures no
                                    // objects are cropped
  pass.filter(*cloud_in);

  if (cloud_in->empty()) {
    ROS_INFO_STREAM("Empty cloud z!");
  }

  pcl::transformPointCloud(*cloud_in, *cloud_out, eigen_tf.inverse());
  ROS_INFO_STREAM("Filtered");
}

bool splitCloudCallback(apc_msgs::DoSegmentation::Request &req,
                        apc_msgs::DoSegmentation::Response &res) {
  PointCloud::Ptr cloud_(new PointCloud);
  PointCloud::Ptr cloud_out(new PointCloud);
  std::string frame_id;
  ref_tf = req.input_cloud.header.frame_id;
  ROS_INFO_STREAM(ref_tf);

  pcl::fromROSMsg(req.input_cloud, *cloud_);

  if (cloud_->empty()) {
    ROS_INFO_STREAM("Empty cloud!");
  }

  if (current_bin.compare("tote") == 0) {
    ROS_INFO_STREAM("Realsense, tote crop");

    splitCloudTote(cloud_, cloud_out);
    frame_id = "camera_rgb_optical_frame";

  } else if (current_bin.compare("kinect") == 0) {
    ROS_INFO_STREAM("Kinect crop");

    splitCloudKinect(cloud_, cloud_out);
    frame_id = "kinect2_rgb_optical_frame";

  } else {
    ROS_INFO_STREAM("Realsense, bin crop");
    splitCloud(cloud_, cloud_out);
    frame_id = "camera_rgb_optical_frame";
  }
  if (cloud_->empty()) {
    ROS_INFO_STREAM("Empty cloud!");
    return false;
  }
  Eigen::Affine3d eigen_tf;
  tf::StampedTransform transform;

  lookupTransform(frame_id, ref_tf, transform);

  tf::transformTFToEigen(transform, eigen_tf);

  pcl::transformPointCloud(*cloud_out, *cloud_out, eigen_tf);

  pcl::toROSMsg(*cloud_out, res.segmented_cloud);
  res.segmented_cloud.header.stamp = ros::Time::now();
  res.segmented_cloud.header.frame_id = frame_id;
  point_pub.publish(res.segmented_cloud);
  ROS_INFO_STREAM(res.segmented_cloud.header.frame_id);
  return true;
}

void points_callback(const sensor_msgs::PointCloud2ConstPtr &cloud_ptr) {
  if (publish_) {
    PointCloud::Ptr cloud_(new PointCloud);

    sensor_msgs::PointCloud2 crop_msg;
    crop_msg.header = cloud_ptr->header;

    pcl::fromROSMsg(*cloud_ptr, *cloud_);

    splitCloud(cloud_, cloud_);

    pcl::toROSMsg(*cloud_, crop_msg);
    point_pub.publish(crop_msg);
  }
}

void camera_info_callback(const sensor_msgs::CameraInfo &msg) {
  image_width = msg.width;
  image_height = msg.height;
}

void image_callback(const sensor_msgs::ImageConstPtr &msg) {
  // ROS_INFO_STREAM("Image Callback");

  if (stop) {
    return;
  }
  if (inloop_image) {
    return;
  }
  inloop_image = true;
  updatePoints();
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception &e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  // Initialise a mask for the direction
  Mat mask = cv::Mat(image_height, image_width, CV_8UC1, Scalar(0));
  // Fill the mask with the points that have been sent via TODO
  vector<vector<Point>> pts = {{TL_points, TR_points, BR_points, BL_points}};
  fillPoly(mask, pts, Scalar(255));

  // Create a Mat container for the output image
  Mat Color_cropped;
  // temp lab channels for the input and output image
  vector<cv::Mat> lab_channels;
  vector<cv::Mat> lab_channels_dest;
  // split the input image into 3 channels
  RGB_image = cv_ptr->image;
  cv::split(RGB_image, lab_channels);

  for (int i = 0; i < 3; i++) {
    // loop over each channel and cutout the mask.
    lab_channels_dest.push_back(cv::Mat(1080, 1920, CV_8UC1, Scalar(0)));
    bitwise_and(lab_channels[i], mask, lab_channels_dest[i]);
  }
  cv::merge(lab_channels_dest, Color_cropped);

  // // if (DEBUG) {
  // // Update GUI Window
  // cv::imshow(OPENCV_WINDOW, Color_cropped);
  // cv::waitKey(3);
  // // }
  cv_bridge::CvImage out_msg;
  out_msg.header = cv_ptr->header; // Same timestamp and tf frame as input image
  out_msg.encoding = cv_ptr->encoding; // Or whatever
  out_msg.image = Color_cropped;       // Your cv::Mat
  // Output modified video stream
  image_pub_.publish(out_msg.toImageMsg());
  inloop_image = false;
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "shelf_based_image_cropper_node");
  // create a node handler for the ros interface
  ros::NodeHandle nh_("~");
  ros::NodeHandle nh;

  std::string pc_topic;
  nh.getParam("/shelf_based_image_cropper_node/pc_topic", pc_topic);
  nh.getParam("/shelf_based_image_cropper_node/ref_tf", ref_tf);

  nh.getParam("/shelf_based_image_cropper_node/publish", publish_);
  ROS_INFO_STREAM("Publish: " << publish_);
  if (pc_topic.empty()) {
    pc_topic = "/realsense/points_aligned";
  }
  if (ref_tf.empty()) {
    ref_tf = "/camera_rgb_optical_frame";
  }

  ROS_INFO_STREAM("Pointcloud Topic: " << pc_topic);
  ROS_INFO_STREAM("Camera TF: " << ref_tf);

  tf_listener_ptr.reset(new tf::TransformListener());

  image_transport::ImageTransport it_(nh_);

  // advertise cropped feeds
  image_pub_ = it_.advertise("/realsense/rgb/image_raw_cropped", 1);
  point_pub = nh_.advertise<sensor_msgs::PointCloud2>(
      "/realsense/points_aligned_cropped", 1);
  // make a service call for the change of value
  ros::ServiceServer change_bin_service =
      nh_.advertiseService("/apc_3d_vision/split_cloud_change_bin", changeBin);
  ros::ServiceServer split_service =
      nh_.advertiseService("/apc_3d_vision/split_cloud", splitCloudCallback);
  // load in the depth and the rgb images
  // imageSub_ = nh_.subscribe("/realsense/rgb/image_raw", 1, image_callback);
  camera_info_sub =
      nh_.subscribe("/realsense/rgb/camera_info", 1, camera_info_callback);
  pointSub_ = nh_.subscribe(pc_topic, 1, points_callback);

  nh.getParam("shelf_layout", shelf_layout_);

  // cv::namedWindow(OPENCV_WINDOW);
  // if (CALIBRATE) {
  //     char TrackbarName[50];
  //     sprintf(TrackbarName, "Width x %d", valueMax);
  //     cv::createTrackbar(TrackbarName, OPENCV_WINDOW, &value, valueMax,
  //                        ChangeWidth);
  //
  //     char TrackbarName2[50];
  //     sprintf(TrackbarName2, "Height x %d", valueMax);
  //     cv::createTrackbar(TrackbarName2, OPENCV_WINDOW, &value,
  //     valueMax,
  //                        ChangeHeight);
  //     char TrackbarName3[50];
  //     sprintf(TrackbarName3, "WidthOffset x %d", valueMax);
  //     cv::createTrackbar(TrackbarName3, OPENCV_WINDOW, &value,
  //     valueMax,
  //                        ChangeWidthOffset);
  //
  //     char TrackbarName4[50];
  //     sprintf(TrackbarName4, "HeightOffset x %d", valueMax);
  //     cv::createTrackbar(TrackbarName4, OPENCV_WINDOW, &value,
  //     valueMax,
  //                        ChangeHeightOffset);
  // }
  // std::cout << "entering ros spin" << std::endl;
  stop = false;
  ros::spin();
  return 0;
}

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
