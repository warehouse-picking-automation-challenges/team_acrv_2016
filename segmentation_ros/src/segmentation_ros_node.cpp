#include <pcl/filters/filter.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <string>

#include <apc_msgs/DoSegmentation.h>
#include <segmentation/segmentation.hpp>

#include <apc_msgs/DoSegmentationAction.h>
#include <apc_msgs/EnablePublisher.h>

#include <actionlib/server/simple_action_server.h>

#include <boost/thread.hpp>

/**
*Node for segmentation of 3D points using segmentation library
*/

using namespace std;
using namespace pcl;
using namespace ros;
using namespace APC;

class SegmentationNode {
   public:
    SegmentationNode(ros::NodeHandle nh)
        : nh_(nh),
          publisherEnabled_(true),
          actionServer_(nh_, "segmentation", false) {
        std::string pc_topic;
        nh_.getParam("pointcloud_topic", pc_topic);

        if (pc_topic.empty()) {
            pc_topic = "/realsense/points_aligned";
        }
        // pub sub
        pcl_sub_ =
            nh_.subscribe(pc_topic, 1, &SegmentationNode::scan_callback, this);
        segmented_pub_ =
            nh_.advertise<sensor_msgs::PointCloud2>("segmented_pointcloud", 1);

        // register services
        doSegmentationSrv_ = nh_.advertiseService(
            "do_segmentation", &SegmentationNode::doSegmentation, this);
        enablePublisherSrv_ = nh_.advertiseService(
            "enable_publisher", &SegmentationNode::enablePublisher, this);

        // register action server
        actionServer_.registerGoalCallback(
            boost::bind(&SegmentationNode::goalCB, this));

        actionServer_.start();

        Config config = segmentation.getConfig();

        // Initialize params from the param server
        nh_.param<float>("voxel_resolution", config.voxel_resolution, 0.003f);
        nh_.param<float>("seed_resolution", config.seed_resolution, 0.05f);

        nh_.param<float>("color_importance", config.color_importance, 1.0f);
        nh_.param<float>("spatial_importance", config.spatial_importance, 0.4f);
        nh_.param<float>("normal_importance", config.normal_importance, 1.0f);

        nh_.param<bool>("use_single_cam_transform",
                        config.use_single_cam_transform, false);
        nh_.param<bool>("use_supervoxel_refinement",
                        config.use_supervoxel_refinement, false);

        nh_.param<bool>("use_random_sampling", config.use_random_sampling,
                        false);
        nh_.param<float>("outlier_cost", config.outlier_cost, 0.02f);
        nh_.param<float>("smooth_cost", config.smooth_cost,
                         config.outlier_cost * 0.01);

        nh_.param<int>("min_inliers_per_plane", config.min_inliers_per_plane,
                       10);
        nh_.param<float>(
            "label_cost", config.label_cost,
            config.min_inliers_per_plane * 0.5 * config.outlier_cost);

        nh_.param<int>("max_num_iterations", config.max_num_iterations, 25);
        nh_.param<float>("max_curvature", config.max_curvature, 0.001f);
        nh_.param<int>("gc_scale", config.gc_scale, 1e4);

        segmentation.setConfig(config);
    }

    /**
    * @brief goalCB callback funtion for the new goal
    */
    void goalCB() {
        actionServer_.acceptNewGoal();
        publisherEnabled_ = false;
        segmentation_thread = new boost::thread(
            boost::bind(&SegmentationNode::doSegmentation, this));
    }

    /**
    * @brief doSegmentation thread for doing the segmentation
    * @param cloud_ptr
    * @return
    */
    void doSegmentation() {
        apc_msgs::DoSegmentationResult result;
        pcl::PointCloud<pcl::PointXYZL>::Ptr segmented_pc_ptr;

        std::vector<int> removedIndices;
        pcl::removeNaNFromPointCloud(cloud_, cloud_, removedIndices);

        if (cloud_.size() > 0) {
            segmentation.setPointCloud(cloud_.makeShared());
            segmentation.doSegmentation();
            segmented_pc_ptr = segmentation.getSegmentedPointCloud();

            pcl::PointCloud<PointXYZL> cloud2_;

            pcl::copyPointCloud(*segmented_pc_ptr, cloud2_);
            cloud2_.clear();

            BOOST_FOREACH (pcl::PointXYZL point, *segmented_pc_ptr) {
                if (point.label == 0) continue;
                cloud2_.push_back(point);
            }

            pcl::toROSMsg(cloud2_, result.segmented_cloud);

            actionServer_.setSucceeded(result);
        } else {
            ROS_ERROR("No points in cloud. Aborting..\n");
            actionServer_.setAborted(result);
        }
    }

    /**
    * @brief enablePublisher
    * @param req
    * @param resp
    * @return
    */
    bool enablePublisher(apc_msgs::EnablePublisher::Request &req,
                         apc_msgs::EnablePublisher::Response &resp) {
        ROS_INFO("Publisher set to %d\n", req.enable);
        publisherEnabled_ = req.enable;
        return true;
    }

    /**
    * @brief doSegmentation service callback for synchronized processing
    * @param req the input point cloud
    * @param res the output point cloud
    * @return true if the service succeeded
    */
    bool doSegmentation(apc_msgs::DoSegmentation::Request &req,
                        apc_msgs::DoSegmentation::Response &res) {
        sensor_msgs::PointCloud2 seg_msg;
        pcl::PointCloud<pcl::PointXYZL>::Ptr segmented_pc_ptr;

        pcl::fromROSMsg(req.input_cloud, cloud_);

        std::vector<int> removedIndices;
        pcl::removeNaNFromPointCloud(cloud_, cloud_, removedIndices);

        ROS_INFO("Got cloud %lu points\n", cloud_.size());

        segmentation.setPointCloud(cloud_.makeShared());

        segmentation.doSegmentation();

        segmented_pc_ptr = segmentation.getSegmentedPointCloud();

        pcl::PointCloud<PointXYZL> cloud2_;
        if (cloud_.size() > 0) {
            pcl::copyPointCloud(*segmented_pc_ptr, cloud2_);
            cloud2_.clear();

            BOOST_FOREACH (pcl::PointXYZL point, *segmented_pc_ptr) {
                if (point.label == 0) continue;
                cloud2_.push_back(point);
            }
            pcl::toROSMsg(cloud2_, res.segmented_cloud);
            res.segmented_cloud.header = req.input_cloud.header;
            segmented_pub_.publish(res.segmented_cloud);
        } else {
            res.segmented_cloud.header = req.input_cloud.header;
        }
        return true;
    }
    /**
    * @brief scan_callback
    * @param cloud_ptr
    */
    void scan_callback(const sensor_msgs::PointCloud2ConstPtr &cloud_ptr) {
        pcl::fromROSMsg(*cloud_ptr, cloud_);

        if (publisherEnabled_ && cloud_.size() > 0) {
            sensor_msgs::PointCloud2 seg_msg;
            pcl::PointCloud<pcl::PointXYZL>::Ptr segmented_pc_ptr;

            std::vector<int> removedIndices;
            pcl::removeNaNFromPointCloud(cloud_, cloud_, removedIndices);

            segmentation.setPointCloud(cloud_.makeShared());

            segmentation.doSegmentation();

            segmented_pc_ptr = segmentation.getSegmentedPointCloud();

            if (cloud_.size() > 0) {
                pcl::PointCloud<PointXYZL> cloud2_;

                pcl::copyPointCloud(*segmented_pc_ptr, cloud2_);
                cloud2_.clear();

                BOOST_FOREACH (pcl::PointXYZL point, *segmented_pc_ptr) {
                    if (point.label == 0) continue;
                    cloud2_.push_back(point);
                }

                pcl::toROSMsg(cloud2_, seg_msg);
                segmented_pub_.publish(seg_msg);
            }
        }
    }

   private:
    ros::NodeHandle nh_;

    // pub sub
    ros::Publisher segmented_pub_;
    ros::Subscriber pcl_sub_;

    // services
    ros::ServiceServer doSegmentationSrv_;
    ros::ServiceServer enablePublisherSrv_;

    // action libs
    actionlib::SimpleActionServer<apc_msgs::DoSegmentationAction> actionServer_;

    boost::thread *segmentation_thread;

    pcl::PointCloud<PointT> cloud_;
    Segmentation segmentation;

    // enable/desable publisher
    bool publisherEnabled_;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "segmentation_node");
    ros::NodeHandle nh("~");
    SegmentationNode node(nh);

    ros::spin();

    ROS_INFO("Terminating node...");

    return 0;
}
