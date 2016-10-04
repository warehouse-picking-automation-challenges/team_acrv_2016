#include "ros_kinfu.h"
#include <ctime>

using namespace std;
using namespace pcl;
using namespace pcl::gpu;
using namespace Eigen;
namespace pc = pcl::console;

ros_kinfu::ros_kinfu(boost::shared_ptr<CameraPoseProcessor> pose_processor)
        : nh_("~"),
        it_(nh_),
        exit_(false),
        scan_(false),
        scan_mesh_(false),
        scan_volume_(false),
        scene_cloud_view_(false),
        kinfu_(480, 640),
        image_view_(false),
        independent_camera_(false),
        pcd_source_(false),
        time_ms_(0),
        pose_processor_(pose_processor),
        kinfu_publisher(nh_)

{
        nh_.param("depth_image_topic", topicDepth,
                  std::string("/camera/depth/image_registered"));
        ROS_INFO_STREAM("Listening for depth on topic: " << topicDepth);
        nh_.param("rgb_image_topic", topicColor,
                  std::string("/camera/rgb/image_registered"));
        ROS_INFO_STREAM("Listening for colour on topic: " << topicColor);
        nh_.param("camera_info_topic", topicCameraInfo,
                  std::string("/camera/depth/camera_info"));
        nh_.param("kinfu_reset_topic", topicReset, std::string("/ros_kinfu/reset"));
        nh_.param("kinfu_pause_topic", topicPause, std::string("/ros_kinfu/pause"));

        nh_.param("publish_points", publish_, false);
        nh_.param("visualise", viz_, false);
        nh_.param("use_hints", use_hints_, true);
        nh_.param("registration", registration_, false);
        nh_.param("integrate_colors", integrate_colors_, true);
        nh_.param("update_kinect_world_frame", update_kinect_world_, true);

        nh_.param("camera_frame_id", camera_frame_id,
                  std::string("/camera_depth_optical_frame"));

        double vsz, tsdf_trunc_, icp_weight, cam_move_threshold;
        int height, width, device;

        nh_.param("volume_size", vsz, 0.5);
        nh_.param("camera_fx", fx_, 478.507); // new: 478.507, old: 463.888885
        nh_.param("camera_fy", fy_, 478.507);
        nh_.param("camera_cx", cx_, 320.0);
        nh_.param("camera_cy", cy_, 240.0);

        nh_.param("tsdf_trunc", tsdf_trunc_, 0.005);
        nh_.param("icp_weight", icp_weight, 0.01);
        nh_.param("camera_movement_threshold", cam_move_threshold, 0.001);
        nh_.param("image_height", height, 480);
        nh_.param("image_width", width, 640);

        nh_.param("gpu_device", device, 0);

        pcl::gpu::setDevice(device);
        pcl::gpu::printShortCudaDeviceInfo(device);

        // Init Kinfu Tracker
        Eigen::Vector3f volume_size = Vector3f::Constant(vsz /*meters*/);
        kinfu_.volume().setSize(volume_size);
        Eigen::Matrix3f R = Eigen::Matrix3f::Identity();
        Eigen::Vector3f t = volume_size * 0.5f - Vector3f(0, 0, volume_size(2) / 2);

        Eigen::Affine3f pose = Eigen::Translation3f(t) * Eigen::AngleAxisf(R);

        kinfu_.setInitalCameraPose(pose);

        kinfu_.volume().setTsdfTruncDist(tsdf_trunc_ /*meters*/);
        kinfu_.setIcpCorespFilteringParams(icp_weight /*meters*/,
                                           sin(pcl::deg2rad(20.f)));
        kinfu_.setCameraMovementThreshold(cam_move_threshold);

        //    if (!icp)
        //      kinfu_.disableIcp();

        // Init KinfuApp
        tsdf_cloud_ptr_ = PointCloudTSDF::Ptr(new PointCloudTSDF);
        image_view_.raycaster_ptr_ = RayCaster::Ptr(
                new RayCaster(kinfu_.rows(), kinfu_.cols(), fx_, fy_, cx_, cy_));

        if (viz_) {
                scene_cloud_view_.cloud_viewer_->registerKeyboardCallback(
                        keyboard_callback, (void*)this);
                image_view_.viewerScene_->registerKeyboardCallback(keyboard_callback,
                                                                   (void*)this);
                // image_view_.viewerDepth_->registerKeyboardCallback
                // (keyboard_callback, (void*)this);
                scene_cloud_view_.toggleCube(volume_size);
        }

        kinfu_.setDepthIntrinsics(fx_, fy_, cx_, cy_);

        if (integrate_colors_) {
                const int max_color_integration_weight = 2;
                kinfu_.initColorIntegration(max_color_integration_weight);
        }

        colorImageSubscriberFilter =
                new image_transport::SubscriberFilter(it_, topicColor, 1);
        depthImageSubscriberFilter =
                new image_transport::SubscriberFilter(it_, topicDepth, 1);

        cameraInfoSubscriber =
                nh_.subscribe(topicCameraInfo, 1, &ros_kinfu::cameraInfoCallback, this);

        resetSubscriber =
                nh_.subscribe(topicReset, 1, &ros_kinfu::resetCallback, this);

        pauseSubscriber =
                nh_.subscribe(topicPause, 1, &ros_kinfu::pauseCallback, this);

        sync = new message_filters::Synchronizer<SyncPolicy>(
                SyncPolicy(2), *colorImageSubscriberFilter,
                *depthImageSubscriberFilter);

        service = nh_.advertiseService(
                "/ros_kinfu/get_point_cloud", &ros_kinfu::getCloudCallback, this);

        service_cc = nh_.advertiseService(
                "/ros_kinfu/change_camera", &ros_kinfu::changeCameraCallback, this);

}

ros_kinfu::~ros_kinfu() {
        delete &kinfu_;
        delete &depth_device_;
        delete &generated_depth_;
        delete &scene_cloud_view_;
        delete &image_view_;
        delete &tsdf_volume_;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void ros_kinfu::initCurrentFrameView() {
        current_frame_cloud_view_ =
                boost::shared_ptr<CurrentFrameCloudView>(new CurrentFrameCloudView());
        current_frame_cloud_view_->cloud_viewer_.registerKeyboardCallback(
                keyboard_callback, (void*)this);
        current_frame_cloud_view_->setViewerPose(kinfu_.getCameraPose());
}

void ros_kinfu::initRegistration() {
        registration_ = true;
}

void ros_kinfu::setDepthIntrinsics() {
        kinfu_.setDepthIntrinsics(fx_, fy_, cx_, cy_);
        cout << "Depth intrinsics changed to fx=" << fx_ << " fy=" << fy_
             << " cx=" << cx_ << " cy=" << cy_ << endl;
}

void ros_kinfu::cameraInfoCallback(const sensor_msgs::CameraInfo& msg) {
        if (intrins_unset) {
                fx_ = (float)msg.K[0];
                fy_ = (float)msg.K[4];
                cx_ = (float)msg.K[2];
                cy_ = (float)msg.K[5];

                ros_kinfu::setDepthIntrinsics();
                intrins_unset = false;
        }
}

bool ros_kinfu::changeCameraCallback(std_srvs::Empty::Request &req,
                                     std_srvs::Empty::Response &res) {

    nh_.param("depth_image_topic", topicDepth,
              std::string("/camera/depth/image_registered"));
    ROS_INFO_STREAM("Listening for depth on topic: " << topicDepth);
    nh_.param("rgb_image_topic", topicColor,
              std::string("/camera/rgb/image_registered"));
    ROS_INFO_STREAM("Listening for colour on topic: " << topicColor);
    nh_.param("camera_info_topic", topicCameraInfo,
              std::string("/camera/depth/camera_info"));

    nh_.param("camera_frame_id", camera_frame_id,
              std::string("/camera_depth_optical_frame"));
    double vsz;
    nh_.param("volume_size", vsz, 0.5);
    Eigen::Vector3f volume_size = Vector3f::Constant(vsz /*meters*/);
    kinfu_.volume().setSize(volume_size);
    colorImageSubscriberFilter =
            new image_transport::SubscriberFilter(it_, topicColor, 1);
    depthImageSubscriberFilter =
            new image_transport::SubscriberFilter(it_, topicDepth, 1);

    cameraInfoSubscriber =
            nh_.subscribe(topicCameraInfo, 1,
                &ros_kinfu::cameraInfoCallback, this);


    sync = new message_filters::Synchronizer<SyncPolicy>(
            SyncPolicy(2), *colorImageSubscriberFilter,
            *depthImageSubscriberFilter);

    intrins_unset = true;
            reset_command_ = true;
            pause_command_ = false;

    return true;
}

void ros_kinfu::setDepthIntrinsics(std::vector<float> depth_intrinsics) {
        float fx = depth_intrinsics[0];
        float fy = depth_intrinsics[1];

        if (depth_intrinsics.size() == 4) {
                float cx = depth_intrinsics[2];
                float cy = depth_intrinsics[3];
                kinfu_.setDepthIntrinsics(fx, fy, cx, cy);
                cout << "Depth intrinsics changed to fx=" << fx << " fy=" << fy
                     << " cx=" << cx << " cy=" << cy << endl;
        } else {
                kinfu_.setDepthIntrinsics(fx, fy);
                cout << "Depth intrinsics changed to fx=" << fx << " fy=" << fy << endl;
        }
}

void ros_kinfu::toggleColorIntegration() {
        if (registration_) {
                const int max_color_integration_weight = 2;
                kinfu_.initColorIntegration(max_color_integration_weight);
                integrate_colors_ = true;
        }
        cout << "Color integration: "
             << (integrate_colors_ ? "On" : "Off ( requires registration mode )")
             << endl;
}

void ros_kinfu::enableTruncationScaling() {
        kinfu_.volume().setTsdfTruncDist(kinfu_.volume().getSize() (0) / 100.0f);
}

void ros_kinfu::toggleIndependentCamera() {
        independent_camera_ = !independent_camera_;
        cout << "Camera mode: "
             << (independent_camera_ ? "Independent" : "Bound to Kinect pose")
             << endl;
}

void ros_kinfu::togglePublisher() {
        publish_ = !publish_;
        cout << "TSDF Publisher: "
             << (publish_ ? "TSDF Publisher On" : "TSDF Publisher Off ") << endl;
}

void ros_kinfu::resetCallback(const std_msgs::Empty& /*msg*/) {
        reset_command_ = true;
        pause_command_ = false;
}

void ros_kinfu::pauseCallback(const std_msgs::Empty& /*msg*/) {
        pause_command_ = true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void ros_kinfu::reset_kf_world() {
        bool tf_success = false;
        tf::StampedTransform stamped_transform, kinect_transform;
        try {
                cout << "Waiting for Transform" << endl;
                tf_success = tf_listener.waitForTransform(
                        "/world", camera_frame_id, ros::Time(0), ros::Duration(0.1));
                if (tf_success) {
                        tf_listener.lookupTransform("/world", camera_frame_id, ros::Time(0),
                                                    stamped_transform);
                        tf_listener.lookupTransform(kinfu_publisher.kf_world_frame,
                                                    camera_frame_id, ros::Time(0),
                                                    kinect_transform);
                        kinfu_publisher.updateKinectWorldTransform(stamped_transform,
                                                                   kinfu_.getCameraPose());
                        kinfu_publisher.publishKfWorldTransform();
                }
                // kinfu_publisher.updateKinectWorldTransform(stamped_transform,kinfu_.getCameraPose());
        } catch (tf::TransformException ex) {
                ROS_ERROR("%s", ex.what());
                ros::Duration(1.0).sleep();
        }
}

void ros_kinfu::execute(const sensor_msgs::Image::ConstPtr bgr,
                        const sensor_msgs::Image::ConstPtr depth) {

        start = std::clock();
        bool has_image = false;
        bool tf_success = false;
        Affine3f* pose_hint = NULL;
        cv::Mat rgb;

        tf::StampedTransform stamped_transform, kinect_transform;

        cout << "eXECUTING" << endl;

        if (!pause_command_) {
                try {
                        cout << "Waiting for Transform" << endl;
                        tf_success = tf_listener.waitForTransform(
                                "/world", camera_frame_id, ros::Time(0), ros::Duration(0.1));
                        if (tf_success) {
                                tf_listener.lookupTransform("/world", camera_frame_id,
                                                            ros::Time(0), stamped_transform);
                                tf_listener.lookupTransform(kinfu_publisher.kf_world_frame,
                                                            camera_frame_id, ros::Time(0),
                                                            kinect_transform);
                        }

                } catch (tf::TransformException ex) {
                        ROS_ERROR("%s", ex.what());
                        ros::Duration(1.0).sleep();
                }

                if (reset_command_) {
                        kinfu_.reset();
                        reset_kf_world();

                        // update kinect world transform
                        if (update_kinect_world_) {
                                kinfu_publisher.updateKinectWorldTransform(
                                        stamped_transform, kinfu_.getCameraPose());
                        }

                        reset_command_ = false;
                        ROS_INFO("KinFu was reset.");
                }

                depth_device_.upload(&(depth->data[0]), depth->step, depth->height,
                                     depth->width);

                if (integrate_colors_) {
                        readImageRGB(bgr, rgb);
                        image_view_.colors_device_.upload(rgb.data, rgb.step, rgb.rows,
                                                          rgb.cols);
                }

                if (use_hints_) {
                        Affine3d kinect_camera_pose_hint;

                        tf::transformTFToEigen(kinect_transform, kinect_camera_pose_hint);

                        pose_hint = new Affine3f(kinect_camera_pose_hint);

                        cout << "Found Pose" << endl;
                        // run kinfu algorithm
                        if (integrate_colors_)
                                has_image = kinfu_.operator()(
                                        depth_device_, image_view_.colors_device_, pose_hint);
                        else
                                has_image = kinfu_.operator()(depth_device_);

                } else {
                        // run kinfu algorithm
                        if (integrate_colors_)
                                has_image = kinfu_.operator()(depth_device_,
                                                              image_view_.colors_device_);
                        else
                                has_image = kinfu_.operator()(depth_device_);
                }

                // process camera pose
                if (pose_processor_) {
                        pose_processor_->processPose(kinfu_.getCameraPose());
                }
        }

        // JAMES modified the rest of this function - always publish poses,
        // selective publishing of pointcloud

        // if(new_tsdf_) kinfu_publisher.publishTSDFCloud(tsdf_cloud_ptr_);
        // kinfu_publisher.publishCameraPose(kinfu_.getCameraPose());
        kinfu_publisher.publishTFfromPose(kinfu_.getCameraPose());
        kinfu_publisher.publishKfWorldTransform();
        // image_view_.generateDepth(kinfu_,
        // kinfu_.getCameraPose(),generated_depth_);
        // kinfu_publisher.publishDepth(generated_depth_);

        if (publish_) {
                scene_cloud_view_.generateCloud(kinfu_, integrate_colors_);
                if (integrate_colors_) {
                        kinfu_publisher.publishCloud(scene_cloud_view_.cloud_ptr_);
                        kinfu_publisher.publishColorCloud(
                                scene_cloud_view_.combined_color_ptr_);
                } else {
                        kinfu_publisher.publishCloud(scene_cloud_view_.cloud_ptr_);
                }
        }

        double duration = ( std::clock() - start )*1000 / (double) CLOCKS_PER_SEC;
        ROS_INFO_STREAM("Execute duration: " << duration << "ms.");
}

// JAMES added this function as a service callback for requesting the pointcloud
// currently only works if publish_ = true, why?

bool ros_kinfu::getCloudCallback(apc_msgs::GetKinfuPointCloud::Request &req,
                                 apc_msgs::GetKinfuPointCloud::Response &res){

        scene_cloud_view_.generateCloud(kinfu_, integrate_colors_);
        if (integrate_colors_) {
                pcl::toROSMsg(*(scene_cloud_view_.combined_color_ptr_),res.point_cloud);
                kinfu_publisher.publishColorCloud(
                        scene_cloud_view_.combined_color_ptr_);
        } else {
                pcl::toROSMsg(*(scene_cloud_view_.cloud_ptr_),res.point_cloud);
        }

        res.success.data = true;
        return true;
}

void ros_kinfu::readImageRGB(const sensor_msgs::Image::ConstPtr msgImage,
                             cv::Mat& image) {
        cv_bridge::CvImageConstPtr pCvImage;
        pCvImage =
                cv_bridge::toCvShare(msgImage, sensor_msgs::image_encodings::RGB8);
        pCvImage->image.copyTo(image);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void ros_kinfu::startMainLoop() {
        tf::StampedTransform stamped_transform;
        try {
                bool success = tf_listener.waitForTransform(
                        "/world", camera_frame_id, ros::Time(0), ros::Duration(1));
                if (success) {
                        cout << "Initialising Kinect World Frame at current camera location"
                             << endl;
                        tf_listener.lookupTransform("/world", camera_frame_id, ros::Time(0),
                                                    stamped_transform);
                        kinfu_publisher.updateKinectWorldTransform(stamped_transform,
                                                                   kinfu_.getCameraPose());
                        cout << "blah" << endl;
                        kinfu_publisher.publishKfWorldTransform();
                }

        } catch (tf::TransformException ex) {
                ROS_ERROR("%s", ex.what());
                ros::Duration(1.0).sleep();
        }

        sync->registerCallback(
                boost::bind(&ros_kinfu::execute, this, _1, _2)); // Start subscriber

        bool scene_view_not_stopped =
                viz_ ? !scene_cloud_view_.cloud_viewer_->wasStopped() : true;
        bool image_view_not_stopped =
                viz_ ? !image_view_.viewerScene_->wasStopped() : true;

        while (!exit_ && scene_view_not_stopped && image_view_not_stopped) {
                if (viz_) scene_cloud_view_.cloud_viewer_->spinOnce(3);
                ros::spin();
        }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void ros_kinfu::printHelp() {
        cout << endl;
        cout << "KinFu app hotkeys" << endl;
        cout << "=================" << endl;
        cout << "    H    : print this help" << endl;
        cout << "   Esc   : exit" << endl;
        cout << "    T    : take cloud" << endl;
        cout << "    A    : take mesh" << endl;
        cout << "    M    : toggle cloud exctraction mode" << endl;
        cout << "    N    : toggle normals exctraction" << endl;
        cout << "    I    : toggle independent camera mode" << endl;
        cout << "    B    : toggle volume bounds" << endl;
        cout << "    *    : toggle scene view painting ( requires registration "
        "mode )"
             << endl;
        cout << "    C    : clear clouds" << endl;
        cout << "   1,2,3 : save cloud to PCD(binary), PCD(ASCII), PLY(ASCII)"
             << endl;
        cout << "    7,8  : save mesh to PLY, VTK" << endl;
        cout << "   X, V  : TSDF volume utility" << endl;
        cout << endl;
}

void keyboard_callback(const visualization::KeyboardEvent& e, void* cookie) {
        ros_kinfu* app = reinterpret_cast<ros_kinfu*>(cookie);

        int key = e.getKeyCode();

        if (e.keyUp()) switch (key) {
                case 27:
                        app->exit_ = true;
                        break;
                case (int)'t':
                case (int)'T':
                        app->scan_ = true;
                        break;
                case (int)'a':
                case (int)'A':
                        app->scan_mesh_ = true;
                        break;
                case (int)'h':
                case (int)'H':
                        app->printHelp();
                        break;
                case (int)'m':
                case (int)'M':
                        app->scene_cloud_view_.toggleExtractionMode();
                        break;
                case (int)'n':
                case (int)'N':
                        app->scene_cloud_view_.toggleNormals();
                        break;
                case (int)'c':
                case (int)'C':
                        app->scene_cloud_view_.clearClouds(true);
                        break;
                case (int)'i':
                case (int)'I':
                        app->toggleIndependentCamera();
                        break;
                case (int)'b':
                case (int)'B':
                        app->scene_cloud_view_.toggleCube(
                                app->kinfu_.volume().getSize());
                        break;
                case (int)'p':
                case (int)'P':
                        app->togglePublisher();
                        break;

                //      case (int)'7': case (int)'8': app->writeMesh (key -
                //      (int)'0'); break;
                //      case (int)'1': case (int)'2': case (int)'3': app->writeCloud
                //      (key - (int)'0'); break;
                case '*':
                        app->image_view_.toggleImagePaint();
                        break;

                case (int)'x':
                case (int)'X':
                        app->scan_volume_ = !app->scan_volume_;
                        cout << endl
                             << "Volume scan: "
                             << (app->scan_volume_ ? "enabled" : "disabled") << endl
                             << endl;
                        break;
                case (int)'v':
                case (int)'V':
                        cout << "Saving TSDF volume to tsdf_volume.dat ... " << flush;
                        app->tsdf_volume_.save("tsdf_volume.dat", true);
                        cout << "done [" << app->tsdf_volume_.size() << " voxels]"
                             << endl;
                        cout << "Saving TSDF volume cloud to tsdf_cloud.pcd ... "
                             << flush;
                        pcl::io::savePCDFile<pcl::PointXYZI>(
                                "tsdf_cloud.pcd", *app->tsdf_cloud_ptr_, true);
                        cout << "done [" << app->tsdf_cloud_ptr_->size() << " points]"
                             << endl;
                        break;

                default:
                        break;
                }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int main(int argc, char* argv[]) {
        ros::init(argc, argv, "ros_kinfu");
        // ros::NodeHandle nh("~");

        boost::shared_ptr<CameraPoseProcessor> pose_processor;

        ros_kinfu ros_kinfu_(pose_processor);

        // ros_kinfu_.initRegistration();
        // ros_kinfu_.setDepthIntrinsics();
        // ros_kinfu_.toggleColorIntegration();
        // executing
        try {
                ros_kinfu_.startMainLoop();
        } catch (const pcl::PCLException& /*e*/) {
                cout << "PCLException" << endl;
        } catch (const std::bad_alloc& /*e*/) {
                cout << "Bad alloc" << endl;
        } catch (const std::exception& /*e*/) {
                cout << "Exception" << endl;
        }

        delete &ros_kinfu_;

        return 0;
}
