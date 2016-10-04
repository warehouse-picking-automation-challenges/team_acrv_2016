#include <camera_info_manager/camera_info_manager.h>
#include <cv_bridge/cv_bridge.h>
#include <eigen_conversions/eigen_msg.h>
#include <image_transport/image_transport.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
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

#include <apc_msgs/ChangeBinName.h>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;

ros::Publisher point_pub;
// image_transport::CameraPublisher color_pub;
image_transport::CameraPublisher depth_pub;
ros::Subscriber pointSub_;
ros::Subscriber imageSub_;
image_transport::Publisher image_pub_;
boost::shared_ptr<tf::TransformListener> tf_listener_ptr;

// rs::intrinsics depth_intrin

static const std::string OPENCV_WINDOW = "Cropped Image Window";
static const std::string MY_WINDOW = "color window_yo";
static const std::string OPENCV_WINDOW_depth = "depth window";

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

float c_width = 0.23;
float c_height = 0.23;
float c_widthOF = 0;
float c_heightOF = 0;

bool stop = true;
bool inloop_depth = false;
bool inloop_image = false;
std::string bin_letter = "D";

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

void updatePoints() {
  ros::NodeHandle nh("~");

  std::vector<float> binDim;
  std::string param_str = "/shelf_based_image_cropper_node/bin_";
  param_str.append(bin_letter);
  nh.getParam(param_str, binDim);

  // Set up the new dimensions for the image frame
  // get the focal length of x
  // get the focal length of y
  Eigen::Affine3d camera_to_bin_eigenTL;
  tf::StampedTransform camera_to_bin_TL;

  std::string tf_str = "bin_";
  tf_str.append(bin_letter);
  lookupTransform("/camera_rgb_optical_frame", tf_str, camera_to_bin_TL);
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

  float TL_Xd = TL(0, 3);
  float TL_Yd = TL(1, 3);
  float TL_Zd = TL(2, 3);
  float TR_Xd = TR(0, 3);
  float TR_Yd = TR(1, 3);
  float TR_Zd = TR(2, 3);
  float BL_Xd = BL(0, 3);
  float BL_Yd = BL(1, 3);
  float BL_Zd = BL(2, 3);
  float BR_Xd = BR(0, 3);
  float BR_Yd = BR(1, 3);
  float BR_Zd = BR(2, 3);

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

bool changeBin(apc_msgs::ChangeBinName::Request &req,
               apc_msgs::ChangeBinName::Response &res) {
  bin_letter = req.Bin_Letter.data;

  res.success.data = true;
  stop = false;
  return true;
}

void points_callback(const sensor_msgs::PointCloud2ConstPtr &cloud_ptr) {
  if (stop) {
    return;
  }
  if (inloop_depth) {
    return;
  }
  inloop_depth = true;
  pcl::PointCloud<pcl::PointXYZRGB> cloud_;
  pcl::fromROSMsg(*cloud_ptr, cloud_);

  Mat mask = cv::Mat(480, 640, CV_8UC1, Scalar(0));

  // Fill the mask with the points that have been sent via TODO
  vector<vector<Point>> pts = {
      {TL_points_d, TR_points_d, BR_points_d, BL_points_d}};
  fillPoly(mask, pts, Scalar(255));
  if (DEBUG) {
    cv::namedWindow(MY_WINDOW);
    cv::imshow(MY_WINDOW, mask);
    cv::waitKey(3);
  }

  Mat temp_rgb = cv::Mat(480, 640, CV_8UC3, Scalar(0));
  for (int dy = 0; dy < mask.rows; dy++) {
    for (int dx = 0; dx < mask.cols; dx++) {
      if (uint8_t(0) == mask.at<uchar>(dy, dx)) {
        // this is a empty point
        cloud_[dy * mask.cols + dx].x = 0;
        cloud_[dy * mask.cols + dx].y = 0;
        cloud_[dy * mask.cols + dx].z = 0;
        cloud_[dy * mask.cols + dx].rgba = 0;
      } else {
        if (DEBUG) {
          temp_rgb.at<uchar>(dy, dx, 1) =
              (cloud_[dy * mask.cols + dx].rgba >> 24) & 0x000000ff;
          temp_rgb.at<uchar>(dy, dx, 2) =
              (cloud_[dy * mask.cols + dx].rgba >> 16) & 0x000000ff;
          temp_rgb.at<uchar>(dy, dx, 3) =
              (cloud_[dy * mask.cols + dx].rgba >> 8) & 0x000000ff;
        }
      }
    }
  }

  if (DEBUG) {
    cv::namedWindow("The matrix");
    cv::imshow("The matrix", temp_rgb);
    cv::waitKey(10);
  }
  sensor_msgs::PointCloud2 crop_msg;
  pcl::toROSMsg(cloud_, crop_msg);
  point_pub.publish(crop_msg);
  inloop_depth = false;
}

void image_callback(const sensor_msgs::ImageConstPtr &msg) {
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
  Mat mask = cv::Mat(480, 640, CV_8UC1, Scalar(0));
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
    lab_channels_dest.push_back(cv::Mat(480, 640, CV_8UC1, Scalar(0)));
    bitwise_and(lab_channels[i], mask, lab_channels_dest[i]);
  }
  cv::merge(lab_channels_dest, Color_cropped);

  // if (DEBUG) {
  // Update GUI Window
  cv::imshow(OPENCV_WINDOW, Color_cropped);
  cv::waitKey(3);
  // }
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

  tf_listener_ptr.reset(new tf::TransformListener());

  image_transport::ImageTransport it_(nh_);
  image_transport::ImageTransport image_transport(nh_);

  // advertise cropped feeds
  image_pub_ = it_.advertise("/realsense/rgb/image_raw_cropped", 1);
  point_pub = nh_.advertise<sensor_msgs::PointCloud2>(
      "/realsense/points_aligned_cropped", 1);
  // make a service call for the change of value
  ros::ServiceServer service =
      nh_.advertiseService("Change_Bin_Name", changeBin);
  // load in the depth and the rgb images
  imageSub_ = nh_.subscribe("/realsense/rgb/image_raw", 1, image_callback);
  pointSub_ = nh_.subscribe("/realsense/points_aligned", 1, points_callback);

  cv::namedWindow(OPENCV_WINDOW);
  if (CALIBRATE) {
    char TrackbarName[50];
    sprintf(TrackbarName, "Width x %d", valueMax);
    cv::createTrackbar(TrackbarName, OPENCV_WINDOW, &value, valueMax,
                       ChangeWidth);

    char TrackbarName2[50];
    sprintf(TrackbarName2, "Height x %d", valueMax);
    cv::createTrackbar(TrackbarName2, OPENCV_WINDOW, &value, valueMax,
                       ChangeHeight);
    char TrackbarName3[50];
    sprintf(TrackbarName3, "WidthOffset x %d", valueMax);
    cv::createTrackbar(TrackbarName3, OPENCV_WINDOW, &value, valueMax,
                       ChangeWidthOffset);

    char TrackbarName4[50];
    sprintf(TrackbarName4, "HeightOffset x %d", valueMax);
    cv::createTrackbar(TrackbarName4, OPENCV_WINDOW, &value, valueMax,
                       ChangeHeightOffset);
  }
  std::cout << "entering ros spin" << std::endl;
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
