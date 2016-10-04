/*
Copyright 2016 Australian Centre for Robotic Vision
*/

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_picking_event.h>
#include <boost/shared_ptr.hpp>

#include <eigen_conversions/eigen_msg.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

#include "yaml-cpp/yaml.h"

#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

#include <apc_3d_vision.hpp>

#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

// Global VARIABLES
int ii = 0;  // This is a counter

Apc3dVision apc_vis;

tf::TransformListener* tf_listener;

boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> input_cloud;
boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> clicked_points;
boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> selected_points;
boost::shared_ptr<pcl::PointCloud<pcl::PointXYZL>> labelled_cloud;
boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> shelf;

boost::shared_ptr<pcl::visualization::PCLVisualizer> visualizer;

// pcl::visualization::PCLVisualizer visualizer("PCL visualizer");
std::vector<std::string> axis_names;

double z_bounds;

void pp_callback(const pcl::visualization::PointPickingEvent& event);
// void perform_pca();
double get_axis_average(int axis);
double get_axis_max(int axis);
double get_axis_min(int axis);
bool within_x(double x);
bool within_y(double y);
bool within_z(double z);

void pointsCallback(const sensor_msgs::PointCloud2::ConstPtr msg) {
    pcl::fromROSMsg(*msg, *input_cloud);
}

bool lookupTransform(std::string target_frame, std::string source_frame,
                     tf::Transform* output_transform) {
    // tf::Transform transform;
    tf::StampedTransform stampedtransform;
    bool tf_success = tf_listener->waitForTransform(
        target_frame, source_frame, ros::Time(0), ros::Duration(1.0));
    if (tf_success) {
        tf_listener->lookupTransform(target_frame, source_frame, ros::Time(0),
                                     stampedtransform);
        output_transform->setOrigin(stampedtransform.getOrigin());
        output_transform->setRotation(stampedtransform.getRotation());

        return true;
    } else {
        ROS_INFO("Can't Find Transform from %s to %s!", source_frame.c_str(),
                 target_frame.c_str());
        return false;
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "shelf_registration_ros_node");
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_("~");

    tf_listener = new tf::TransformListener();

    // Allocate Memory
    input_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    shelf.reset(new pcl::PointCloud<pcl::PointXYZ>);
    clicked_points.reset(new pcl::PointCloud<pcl::PointXYZ>);
    selected_points.reset(new pcl::PointCloud<pcl::PointXYZ>);
    labelled_cloud.reset(new pcl::PointCloud<pcl::PointXYZL>);

    bool record_new_shelf_cloud = false;
    std::string saved_shelf_cloud_fn;
    std::string shelf_fn;
    std::string labelled_cloud_fn;
    std::string aligned_shelf_fn;
    std::string found_transform_fn;
    std::string topicPoints = "/kinect2/qhd/points";
    double max_depth;
    double max_right_crop;
    double max_left_crop;

    nh_private_.param<bool>("record_new_shelf_cloud", record_new_shelf_cloud,
                            false);
    nh_private_.param<std::string>(
        "saved_shelf_cloud_fn", saved_shelf_cloud_fn,
        // "package://apc_3d_vision/models/empty_shelf_cloud.pcd");
        "/home/baxter/co/apc_ws/src/apc_3d_vision/models/"
        "empty_shelf_cloud.pcd");
    nh_private_.param<std::string>(
        "shelf_model_fn", shelf_fn,
        // "package://apc_3d_vision/models/pod_lowres.pcd");
        "/home/baxter/co/apc_ws/src/apc_3d_vision/models/pod_lowres.pcd");
    nh_private_.param<std::string>(
        "labelled_cloud_fn", labelled_cloud_fn,
        // "package://apc_3d_vision/models/empty_shelf_labelled.pcd");
        "/home/baxter/co/apc_ws/src/apc_3d_vision/models/"
        "empty_shelf_labelled.pcd");
    nh_private_.param<std::string>(
        "aligned_shelf_fn", aligned_shelf_fn,
        // "package://apc_3d_vision/models/aligned_pod.pcd");
        "/home/baxter/co/apc_ws/src/apc_3d_vision/models/aligned_pod.pcd");
    nh_private_.param<std::string>(
        "found_transform_fn", found_transform_fn,
        // "package://apc_3d_vision/models/aligned_pod.pcd");
        "/home/baxter/co/apc_ws/src/apc_3d_vision/models/"
        "initial_shelf_tf.yaml");
    nh_private_.param<std::string>("points_topic", topicPoints,
                                   "/kinect2/qhd/points");
    nh_private_.param<double>("z_bounds", z_bounds, 0.05);
    nh_private_.param<double>("max_depth", max_depth, 3.0);
    nh_private_.param<double>("max_right_crop", max_right_crop, 3.0);
    nh_private_.param<double>("max_left_crop", max_left_crop, 3.0);

    if (record_new_shelf_cloud) {
        // Subscribe to a point cloud topic
        ros::Subscriber subPoints =
            nh_.subscribe(topicPoints, 10, pointsCallback);

        // Publish on the kinect_toggle topic
        ros::Publisher kinectTogglePub =
            nh_private_.advertise<std_msgs::String>("/kinect_toggle", 1000);

        // Wait for the kinect toggle subscriber node to come online
        while (kinectTogglePub.getNumSubscribers() == 0) {
            printf("Waiting for kinect toggle node to come online...\n");
            ros::Duration(0.5).sleep();
        }

        ros::Rate kinect_toggle_publish_rate(5);  // 5 Hz
        std_msgs::String kinect_toggle_msg;
        kinect_toggle_msg.data = "on";

        kinectTogglePub.publish(kinect_toggle_msg);

        while (nh_.ok()) {
            printf("Waiting for point cloud...\n");
            // Overwrite saved_shelf_cloud
            if (input_cloud->size() > 0) {
                printf("Got me a point cloud...\n");
                apc_vis.save_pcd_file(saved_shelf_cloud_fn, input_cloud);
                break;
            }
            ros::spinOnce();
            kinect_toggle_publish_rate.sleep();
        }

        // Turn off the kinect
        kinect_toggle_msg.data = "off";
        for (int i = 0; i < 10; i++) {
            kinectTogglePub.publish(kinect_toggle_msg);
            kinect_toggle_publish_rate.sleep();
        }
    } else {
        apc_vis.load_pcd_file(saved_shelf_cloud_fn, input_cloud);
    }

    // bool is_success = false;
    // std::string input_cloud_fn = (std::string)argv[1];
    // std::string shelf_fn = (std::string)argv[2];
    // std::string labelled_cloud_fn = (std::string)argv[3];
    // std::string aligned_shelf_fn = (std::string)argv[4];

    // Load the shelf model
    apc_vis.load_pcd_file(shelf_fn, shelf);

    // Transform the shelf model to face the same way as the real shelf
    Eigen::Affine3f transform_a = Eigen::Affine3f::Identity();
    transform_a.rotate(Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitX()));
    Eigen::Affine3f transform_b = Eigen::Affine3f::Identity();
    transform_b.translation() << 0.0, 2.0, 3.0;
    Eigen::Affine3f initial_transform = transform_b * transform_a;
    pcl::transformPointCloud(*shelf, *shelf, initial_transform);

    // for (int i = 0; i < input_cloud->size(); i++) {
    //     if (input_cloud->points[i].z > 3.0) {
    //         printf("Depth is: %f", input_cloud->points[i].z);
    //         input_cloud->points.erase(input_cloud->points.begin()+i);
    //     }
    // }

    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*input_cloud, *input_cloud, indices);

    pcl::PointCloud<pcl::PointXYZ>::iterator itr = (*input_cloud).begin();
    for (; itr != (*input_cloud).end(); ++itr) {
        if (itr->z > max_depth) {
            itr->z = NAN;
        }
        if (itr->x > max_right_crop) {
            itr->x = NAN;
        }
        if (itr->x < max_left_crop) {
            // NOTE make sure max_left_crop is negative
            itr->x = NAN;
        }
    }

    pcl::removeNaNFromPointCloud(*input_cloud, *input_cloud, indices);

    // ---------------------------------- Transform to base frame space

    Eigen::Affine3d transform_d = Eigen::Affine3d::Identity();
    tf::Transform my_transform;

    lookupTransform("/base", "/kinect2_link", &my_transform);

    double roll, pitch, yaw;
    tf::Matrix3x3((my_transform.getRotation())).getRPY(roll, pitch, yaw);
    yaw = 0.0;  // Assume kinect is front-on to the shelf

    my_transform.setRotation(tf::createQuaternionFromRPY(roll, pitch, yaw));

    tf::transformTFToEigen(my_transform, transform_d);

    Eigen::Affine3f transform = transform_d.cast<float>();

    Eigen::Affine3f transform_temp = Eigen::Affine3f::Identity();
    transform_temp.rotate(
        Eigen::AngleAxisf(3.14159 / 2.0, Eigen::Vector3f::UnitX()));
    transform = transform * transform_temp;

    // transform.rotate(
    //     Eigen::AngleAxisf(-M_PI / 180.0f * 34.0f, Eigen::Vector3f::UnitX()));
    // transform.translation() << 0.105, -0.055, 0.940;
    pcl::transformPointCloud(*input_cloud, *input_cloud, transform);
    // ---------------------------------- Transform to base frame space

    // Copy contents of original XYZ Point Cloud to new XYZL Point Cloud
    pcl::copyPointCloud(*input_cloud, *labelled_cloud);

    axis_names.push_back("axis_x");
    axis_names.push_back("axis_y");
    axis_names.push_back("axis_z");

    // Run Visualiser
    visualizer.reset(
        new pcl::visualization::PCLVisualizer("APC Shelf Calibration"));
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
        cloud_color_handler(input_cloud, 255, 255, 255);
    visualizer->addPointCloud(input_cloud, cloud_color_handler,
                              "original_cloud");
    visualizer->registerPointPickingCallback(&pp_callback);
    // visualizer->addCoordinateSystem(1.0f);
    visualizer->spin();

    // -----------------

    std::cout << "Highlighting Points" << std::endl;
    for (int i = 0; i < labelled_cloud->size(); i++) {
        bool is_within_x = within_x(labelled_cloud->points[i].x);
        bool is_within_y = within_y(labelled_cloud->points[i].y);
        bool is_within_z = within_z(labelled_cloud->points[i].z);

        if (is_within_x && is_within_y && is_within_z) {
            labelled_cloud->points[i].label = 1;
            // std::cout << "Point " << i << " labelled.\n";
            selected_points->push_back(pcl::PointXYZ(
                labelled_cloud->points[i].x,
                // get_axis_average(0),
                labelled_cloud->points[i].y, labelled_cloud->points[i].z));
        }
    }

    Apc3dVision::icp_params_t icp_params(selected_points, shelf);

    icp_params.output_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    icp_params.input_cloud_leaf_size = 0.01;
    icp_params.is_downsample_input_cloud = true;
    icp_params.target_cloud_leaf_size = 0.01;
    icp_params.is_downsample_target_cloud = true;
    icp_params.verbose = true;

    apc_vis.align_icp(&icp_params);

    // Perform inverse of the found transform to shift the shelf
    // (CAD model) onto the raw cloud
    pcl::transformPointCloud(*shelf, *shelf, icp_params.transform.inverse());

    // -----------------
    // Inverse transform input_cloud, shelf and labelled_cloud
    pcl::transformPointCloud(*input_cloud, *input_cloud, transform.inverse());
    pcl::transformPointCloud(*labelled_cloud, *labelled_cloud,
                             transform.inverse());
    pcl::transformPointCloud(*shelf, *shelf, transform.inverse());
    // -----------------

    // Visualisation
    pcl::visualization::PCLVisualizer visu1("Alignment");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
        input_colour(input_cloud, 0.0, 255.0, 0.0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
        output_colour(shelf, 255.0, 0.0, 0.0);
    visu1.addPointCloud(input_cloud, input_colour, "raw");
    visu1.addPointCloud(shelf, output_colour, "model");
    visu1.addCoordinateSystem(0.1);
    visu1.spin();

    // Save data to pcd file:
    pcl::io::savePCDFileASCII(aligned_shelf_fn, *shelf);
    pcl::io::savePCDFileASCII(labelled_cloud_fn, *labelled_cloud);
    // pcl::io::savePCDFileASCII(selected_points_fn, *selected_points);
    std::cout << "Aligned shelf cad model saved to file." << std::endl;

    Eigen::Affine3f total_shelf_transform = transform.inverse() *
                                            icp_params.transform.inverse() *
                                            initial_transform;
    std::cout << "Total Shelf Transform\n";
    std::cout << total_shelf_transform.matrix();
    std::cout << std::endl;

    std::vector<float> temp;

    Eigen::MatrixXf temp_matrix = total_shelf_transform.matrix();

    std::stringstream ss;
    ss << "InitialShelfTransform: [";
    for (int row = 0; row < temp_matrix.rows(); row++) {
        for (int col = 0; col < temp_matrix.cols(); col++) {
            if (row == temp_matrix.rows() - 1 &&
                col == temp_matrix.cols() - 1) {
                ss << temp_matrix(row, col) << "]\n";
            } else {
                ss << temp_matrix(row, col) << ", ";
            }
        }
    }

    std::ofstream ofstr;
    ofstr.open(found_transform_fn.c_str());
    ofstr << ss.str().c_str();
    ofstr.close();

    std::cout << "Shelf pre-alignment transform saved to file." << std::endl;

    return EXIT_SUCCESS;
}

void pp_callback(const pcl::visualization::PointPickingEvent& event) {
    std::cout << "Picking event active" << std::endl;

    if (event.getPointIndex() != -1) {
        // Get the point that was picked
        pcl::PointXYZ picked_pt;
        event.getPoint(picked_pt.x, picked_pt.y, picked_pt.z);
        std::cout << "Picked point with index " << ii << ", and coordinates "
                  << picked_pt.x << ", " << picked_pt.y << ", " << picked_pt.z
                  << std::endl;
        ii++;
        (*labelled_cloud)[event.getPointIndex()].label = ii;

        clicked_points->push_back(
            pcl::PointXYZ(picked_pt.x, picked_pt.y, picked_pt.z));

        // Add a sphere to it in the PCLVisualizer window
        std::stringstream ss;
        ss << "sphere_" << ii;
        visualizer->addSphere(picked_pt, 0.01, 1.0, 0.0, 0.0, ss.str());

        if (ii > 3) {
            std::cout << "You've picked enough points now.\n"
                      << "Close the pcl_viewer to continue.\n";
        }
    }
}

double get_axis_average(int axis) {
    double axis_value_sum = 0.0;
    double count = 0.0;
    BOOST_FOREACH (pcl::PointXYZ& point, *clicked_points) {
        axis_value_sum += point.data[axis];
        count++;
    }

    if (count > 0) {
        return axis_value_sum / count;
    } else {
        return 0.0;
    }
}

double get_axis_max(int axis) {
    double axis_max = -DBL_MAX;  // min double value
    BOOST_FOREACH (pcl::PointXYZ& point, *clicked_points) {
        if (point.data[axis] > axis_max) {
            axis_max = point.data[axis];
        }
    }
    // std::cout << "axis_max = " << axis_max << std::endl;
    return axis_max;
}

double get_axis_min(int axis) {
    double axis_min = DBL_MAX;  // min double value
    BOOST_FOREACH (pcl::PointXYZ& point, *clicked_points) {
        if (point.data[axis] < axis_min) {
            axis_min = point.data[axis];
        }
    }
    // std::cout << "axis_min = " << axis_min << std::endl;
    return axis_min;
}

bool within_z(double z) {
    double average_z = get_axis_average(2);
    // double z_bounds = 0.05;

    if (z < (average_z + z_bounds) && z > (average_z - z_bounds)) {
        return true;
    } else {
        return false;
    }
}

bool within_y(double y) {
    double max_y = get_axis_max(1);
    double min_y = get_axis_min(1);

    if (y < max_y && y > min_y) {
        return true;
    } else {
        return false;
    }
}

bool within_x(double x) {
    double max_x = get_axis_max(0);
    double min_x = get_axis_min(0);

    if (x < max_x && x > min_x) {
        return true;
    } else {
        return false;
    }
}

// void perform_pca() {
//     double r = 0.0;
//     double g = 0.0;
//     double b = 0.0;
//
//     // PERFORM PCA
//     pcl::PCA<pcl::PointXYZ> pca;
//     pca.setInputCloud(selected_points);
//     Eigen::Matrix3f eigenvectors_f = pca.getEigenVectors();
//     Eigen::Matrix<double, 3, 3> eigenvectors_d =
//         eigenvectors_f.cast<double>();
//
//     // Ensure z-axis direction satisfies right-hand rule
//     eigenvectors_d.col(2) =
//         eigenvectors_d.col(0).cross(eigenvectors_d.col(1));
//
//     // Calculate the centroid
//     Eigen::Vector4d centroid;
//     pcl::compute3DCentroid(*selected_points, centroid);
//
//
//     for (int i = 0; i < 3; i++) {
//         pcl::PointXYZ axis_start;
//         pcl::PointXYZ axis_end;
//
//         axis_start.x = centroid[0];
//         axis_start.y = centroid[1];
//         axis_start.z = centroid[2];
//         axis_end.x = centroid[0] + eigenvectors_d.col(0)[i];
//         axis_end.y = centroid[1] + eigenvectors_d.col(1)[i];
//         axis_end.z = centroid[2] + eigenvectors_d.col(2)[i];
//
//         int viewport = 0;
//         visualizer.removeShape(axis_names[i], viewport);
//
//         if (i == 0) {r = 1.0; g = 0.0; b = 0.0;}
//         if (i == 1) {r = 0.0; g = 1.0; b = 0.0;}
//         if (i == 2) {r = 0.0; g = 0.0; b = 1.0;}
//
//         visualizer.addLine<pcl::PointXYZ>(
//             axis_start, axis_end, r, g, b, axis_names[i], viewport);
//     }
//
//     std::cout << "Updated PCA Alignment" << std::endl;
// }
