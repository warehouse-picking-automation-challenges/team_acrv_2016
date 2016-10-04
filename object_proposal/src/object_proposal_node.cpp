#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/common/centroid.h>

#include <sensor_msgs/PointCloud2.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <tf/transform_listener.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <image_geometry/pinhole_camera_model.h>


#include <opencv2/highgui/highgui.hpp>

#include <apc_msgs/ObjectProposals.h>
#include <apc_msgs/BoundingBox.h>

#include <apc_msgs/EnablePublisher.h>
#include <apc_msgs/Camera_Info.h>
#include <apc_msgs/DoObjectProposal.h>
#include <apc_msgs/Objects.h>

#include <apc_msgs/GetIDFromImage.h>


#include <boost/foreach.hpp>

#include <sstream>

#include <cmath>

/**
*Node for detecting 3D clusters and align them with corresponding 2D images
*/

using namespace message_filters;
using namespace std;
using namespace pcl;
using namespace cv;
using namespace ros;

typedef struct Proposal{
    cv::Rect bounding_box;
    std::vector<cv::Point2d> imagePoints;
    uint32_t label;
} proposal_t;

class ObjectProposal
{


public:

    ObjectProposal(ros::NodeHandle nh):nh_(nh),camera_initialized_(false),proposalPubEnabled_(true),overlayPubEnabled_(false),objectsPubEnabled_(false){

        pcl_sub_.subscribe(nh_,"/segmentation_node/segmented_pointcloud",1);
        image_sub_.subscribe(nh_,"/realsense/rgb/image_raw",1);

        camera_info_sub_=nh_.subscribe("/realsense/rgb/camera_info",1,&ObjectProposal::onCameraInfo,this);

        proposal_pub_=nh_.advertise<apc_msgs::ObjectProposals>("object_proposals",10);
        overlay_image_pub_=nh_.advertise<sensor_msgs::Image>("overlay_image",1);
        object_pub_=nh_.advertise<apc_msgs::ObjectLabelPose>("objects",1);

        syncPtr_=boost::make_shared<message_filters::Synchronizer<sync_policies::ApproximateTime<sensor_msgs::PointCloud2,sensor_msgs::Image> > >(1000);
        syncPtr_->connectInput(pcl_sub_,image_sub_);
        syncPtr_->registerCallback(boost::bind(&ObjectProposal::scan_callback, this, _1, _2));
        //register services
        enableProposalPubSrv_= nh_.advertiseService("enable_proposal_publisher", &ObjectProposal::enableProposalPub,this);
        enableObjectPubSrv_= nh_.advertiseService("enable_object_publisher", &ObjectProposal::enableObjectPub,this);
        enableOverlayPubSrv_= nh_.advertiseService("enable_overlay_publisher", &ObjectProposal::enableOverlayPub,this);
        doObjectProposalSrv_= nh_.advertiseService("do_proposal", &ObjectProposal::doProposal,this);
        changeCameraInfoSrv_ = nh_.advertiseService("change_camera_info_topic", &ObjectProposal::changeCameraInfo, this);
        //register client
        cnn_client_ = nh_.serviceClient<apc_msgs::GetIDFromImage>("/classifier_node/get_id_from_image");
        file_num_=0;
    }


    /**
    * @brief enablePublisher
    * @param req
    * @param resp
    * @return
    */
    bool enableProposalPub(apc_msgs::EnablePublisher::Request &req, apc_msgs::EnablePublisher::Response &resp){

        ROS_INFO("Proposal publisher set to %d\n",req.enable);
        proposalPubEnabled_=req.enable;
        return true;
    }

    /**
    * @brief enablePublisher
    * @param req
    * @param resp
    * @return
    */
    bool enableObjectPub(apc_msgs::EnablePublisher::Request &req, apc_msgs::EnablePublisher::Response &resp){

        ROS_INFO("Object publisher set to %d\n",req.enable);
        objectsPubEnabled_=req.enable;
        return true;
    }

    bool enableOverlayPub(apc_msgs::EnablePublisher::Request &req, apc_msgs::EnablePublisher::Response &resp){

        ROS_INFO("Overlay publisher set to %d\n",req.enable);
        overlayPubEnabled_=req.enable;
        return true;
    }

    ~ObjectProposal(){}

    /**
    * @brief onCameraInfo gets the intrinsic camara info
    * @param camara_info the info
    */
    void onCameraInfo(const sensor_msgs::CameraInfoConstPtr &camara_info){

        if (camera_initialized_) return;
        else{
            ROS_INFO_STREAM("Camera Initialised!");
            model_.fromCameraInfo(camara_info);
            camera_initialized_=true;
        }

    }

    bool changeCameraInfo(apc_msgs::Camera_Info::Request &req,apc_msgs::Camera_Info::Response &res){

        sensor_msgs::CameraInfo camera_info;
        camera_info = req.camera_info;
        model_.fromCameraInfo(camera_info);
        res.success.data = true;
        return true;

    }


    /**
    * @brief scan_callback Synchronized callback for images and point clouds
    * @param cloud_ptr the cloud
    * @param image_ptr the image
    */
    void scan_callback(const sensor_msgs::PointCloud2ConstPtr &cloud_ptr,const sensor_msgs::ImageConstPtr &image_ptr)
    {

        if (!camera_initialized_) return;

        if ((!objectsPubEnabled_) && (!proposalPubEnabled_) && (!overlayPubEnabled_)) return;

        RNG rng(12345);

        cluster_map_.clear();
        cv_bridge::CvImagePtr cv_ptr;
        cv_bridge::CvImagePtr overlayImage = cv_bridge::CvImagePtr(new cv_bridge::CvImage);
        cv_bridge::CvImagePtr maskImage = cv_bridge::CvImagePtr(new cv_bridge::CvImage);

        vector<proposal_t> proposals;

        pcl::fromROSMsg(*cloud_ptr, labeled_cloud_);

        //image processing block
        try{
            cv_ptr = cv_bridge::toCvCopy(*image_ptr, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e){
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        overlayImage->image=cv_ptr->image.clone();


        BOOST_FOREACH(pcl::PointXYZL point, labeled_cloud_){
            if (point.label == 0) continue;
            //check if cluster map exists for label
            //if not: create it
            std::map<uint32_t,pcl::PointCloud<pcl::PointXYZL> >::iterator iter;
            iter=cluster_map_.find(point.label);
            if (iter == cluster_map_.end()){
                pcl::PointCloud<pcl::PointXYZL> cluster;
                cluster.push_back(point);
                cluster_map_.insert(make_pair(point.label,cluster));
            }//just add the point
            else{
                iter->second.push_back(point);
            }

        }

        std::map<uint32_t,pcl::PointCloud<pcl::PointXYZL> >::iterator iter;

        apc_msgs::ObjectProposals proposals_msg;

        cv::Mat img=cv_ptr->image.clone();


        iter=cluster_map_.begin();

        for (;iter!=cluster_map_.end();iter++){

            proposal_t proposal;
            cv::Mat mask;

            if (this->getBoundingBox(proposal,boost::make_shared<pcl::PointCloud<pcl::PointXYZL> >(iter->second),img,mask)){

                proposal.label=iter->first;

                proposals.push_back(proposal);

                sensor_msgs::PointCloud2 segment_msg;
                pcl::toROSMsg(iter->second,segment_msg);

                apc_msgs::BoundingBox box_msg;
                box_msg.label=iter->first;

                box_msg.top_left.x=proposal.bounding_box.tl().x;
                box_msg.top_left.y=proposal.bounding_box.tl().y;

                box_msg.bottom_right.x=proposal.bounding_box.br().x;
                box_msg.bottom_right.y=proposal.bounding_box.br().y;

                sensor_msgs::Image mask_msg;
                maskImage->image=mask.clone();
                maskImage->encoding=cv_ptr->encoding;
                maskImage->header=cv_ptr->header;
                maskImage->toImageMsg(mask_msg);

                proposals_msg.bounding_boxes.push_back(box_msg);
                proposals_msg.segments.push_back(segment_msg);
                proposals_msg.masks.push_back(mask_msg);

                cv::Scalar colour = cv::Scalar(rng.uniform(0,255),rng.uniform(0,255),rng.uniform(0,255));
                rectangle(overlayImage->image,proposal.bounding_box,colour);

            }

        }

        sensor_msgs::Image overlay_msg;
        overlayImage->encoding=cv_ptr->encoding;
        overlayImage->header=cv_ptr->header;
        overlayImage->toImageMsg(overlay_msg);
        proposals_msg.overlay = overlay_msg;

        if (proposalPubEnabled_){
            proposal_pub_.publish(proposals_msg);
        }

        if (overlayPubEnabled_){
            overlay_image_pub_.publish(overlay_msg);
        }

    }

    /**
    * @brief getBoundingBox gets a projected 2D bounding box around a 3D cluster
    * @param tl top left corner of bounding box
    * @param br bottom right corner of bounding box
    * @param cluster_ptr the pointer to a cluser
    * @param img the RGB image
    * @param mask the output mask for the bounding box
    */

    bool getBoundingBox(proposal_t& proposal, boost::shared_ptr<pcl::PointCloud<pcl::PointXYZL> > cluster_ptr,cv::Mat& img, cv::Mat& mask){

        cv::Mat cluster=cv::Mat::zeros(img.rows,img.cols,img.type());

        bool result;
        //project points to camera plane
        BOOST_FOREACH(pcl::PointXYZL point,*cluster_ptr){
            cv::Point3d xyz=cv::Point3d (point.data[0],point.data[1],point.data[2]);
            cv::Point2d pt=model_.project3dToPixel(xyz);
            if (!isnan(pt.x) && !isnan(pt.y)){
                proposal.imagePoints.push_back(pt);
                int v=static_cast<int>(pt.x);
                int u=static_cast<int>(pt.y);
                if ((v<0)||(u<0))
                // ROS_ERROR("negative values in reprojection. Are the points in the world coordinate frame?\n");
                int donothing=0;
                else
                cluster.at<cv::Vec3b>(u,v)=cv::Vec3b(255,255,255);
            }
        }


        result = this->calcBoundingBox(cluster,proposal.bounding_box);

        //fill mask
        mask = cluster(proposal.bounding_box);
        cv::Mat mask_morphed;
        cv::Mat rgb=img(proposal.bounding_box);
        //invert mask, (255,255,255 0 is masked)
        cv::subtract(cv::Scalar::all(255),mask,mask);
        Mat kernel= cv::getStructuringElement( MORPH_RECT, Size( 5, 5 ), Point( 1, 1) );

        cv::morphologyEx(mask,mask_morphed,cv::MORPH_ERODE,kernel);
        cv::morphologyEx(mask_morphed,mask,cv::MORPH_DILATE,kernel);

       /*
        cv::namedWindow( "mask", WINDOW_AUTOSIZE );// Create a window for display.
        cv::namedWindow( "mask_morphed", WINDOW_AUTOSIZE );// Create a window for display.
        cv::namedWindow( "RGB", WINDOW_AUTOSIZE );// Create a window for display.

        cv::imshow( "mask", mask);
        cv::imshow( "mask_morphed", mask_morphed);
        cv::imshow( "RGB", rgb);
        cv::waitKey(0);
        */
return result;


    }

    /**
    * @brief calcBoundingBox calculates a bounding box around a binary 2D image
    * @param image the binary image
    * @param bb the bounding box
    * @return true if box is found
    */
    bool calcBoundingBox(cv::Mat image, cv::Rect& bb){


        vector<vector<cv::Point> > contours;
        cv::Mat src_gray;

        cv::cvtColor( image, src_gray, CV_BGR2GRAY );
        cv::blur(src_gray, src_gray, cv::Size(3,3) );
        cv::dilate(src_gray, src_gray, cv::Mat(),cv::Point(-1,-1),2);

        cv::findContours( src_gray, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );

        vector<vector<cv::Point> >contours_poly(contours.size());

        //if there are more than one clusters take the largest one
        int i=0;
        int maxSize=0;
        if (contours.size()==0) return false;

        if (contours.size()>1){
            for (int j=0;j<contours.size();j++)
            if (contours[j].size()>maxSize){
                maxSize=contours[j].size();
                i=j;
            }
        }


        cv::approxPolyDP( cv::Mat(contours[i]), contours_poly[i], 3, true );
        bb= cv::boundingRect( cv::Mat(contours_poly[i]) );

        return true;

    }



    /**
    * @brief doProposal service callback for synchronized processing
    * @param req the input point cloud
    * @param res the output point cloud
    * @return true if the service succeeded
    */
    bool doProposal(apc_msgs::DoObjectProposal::Request &req,apc_msgs::DoObjectProposal::Response &res){

        RNG rng(12345);

        if (!camera_initialized_) return false;

        pcl::PointCloud<pcl::PointXYZL > labeled_cloud;
        cv_bridge::CvImagePtr cv_ptr;
        vector<proposal_t> proposals;
        std::map<uint32_t,pcl::PointCloud<pcl::PointXYZL> > cluster_map;


        pcl::fromROSMsg(req.pointcloud_rgb.pointcloud, labeled_cloud);

        //image processing block
        try{
            cv_ptr = cv_bridge::toCvCopy(req.pointcloud_rgb.image, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e){
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return false;
        }


        BOOST_FOREACH(pcl::PointXYZL point, labeled_cloud){
            if (point.label == 0) continue;
            //check if cluster map exists for label
            //if not: create it
            std::map<uint32_t,pcl::PointCloud<pcl::PointXYZL> >::iterator iter;
            iter=cluster_map.find(point.label);
            if (iter == cluster_map.end()){
                pcl::PointCloud<pcl::PointXYZL> cluster;
                cluster.push_back(point);
                cluster_map.insert(make_pair(point.label,cluster));
            }//just add the point
            else{
                iter->second.push_back(point);
            }

        }
        double min_centroid = 1000.0;
        std::map<uint32_t,pcl::PointCloud<pcl::PointXYZL> >::iterator iter;
        cv_bridge::CvImagePtr maskImage = cv_bridge::CvImagePtr(new cv_bridge::CvImage);
        iter=cluster_map.begin();
        for (;iter!=cluster_map.end();iter++){
            proposal_t proposal;
            cv::Mat mask;
            if (this->getBoundingBox(proposal,boost::make_shared<pcl::PointCloud<pcl::PointXYZL> >(iter->second),cv_ptr->image,mask)){

                proposal.label=iter->first;
                proposals.push_back(proposal);

                apc_msgs::BoundingBox box_msg;
                sensor_msgs::PointCloud2 segment_msg;

                pcl::toROSMsg(iter->second,segment_msg);

                segment_msg.header = req.pointcloud_rgb.pointcloud.header;

                box_msg.label=iter->first;

                box_msg.top_left.x=proposal.bounding_box.tl().x;
                box_msg.top_left.y=proposal.bounding_box.tl().y;

                box_msg.bottom_right.x=proposal.bounding_box.br().x;
                box_msg.bottom_right.y=proposal.bounding_box.br().y;

                sensor_msgs::Image mask_msg;
                maskImage->image=mask.clone();
                maskImage->encoding=cv_ptr->encoding;
                maskImage->header=cv_ptr->header;
                maskImage->toImageMsg(mask_msg);

                //storing bounding boxes and the corresponding 3D pointcloud
                res.object_proposals.bounding_boxes.push_back(box_msg);
                res.object_proposals.segments.push_back(segment_msg);
                res.object_proposals.masks.push_back(mask_msg);
                Eigen::Vector4f centroid;
                pcl::compute3DCentroid(iter->second, centroid);

                Eigen::Vector3f view_point(centroid[0], centroid[1], centroid[2]);

                if (sqrt(pow(centroid[0],2) + pow(centroid[1],2)) < min_centroid) {
                    printf("\n\n\n\nI'm in centroid\n\n\n\n");
                    min_centroid = sqrt(pow(centroid[0],2) + pow(centroid[1],2));
                    res.most_central.bounding_boxes.clear();
                    res.most_central.segments.clear();
                    res.most_central.masks.clear();
                    res.most_central.bounding_boxes.push_back(box_msg);
                    res.most_central.segments.push_back(segment_msg);
                    res.most_central.masks.push_back(mask_msg);
                }

                cv::Scalar color = cv::Scalar(rng.uniform(0,255), rng.uniform(0, 255), rng.uniform(0, 255));
                rectangle(cv_ptr->image,proposal.bounding_box, color);

            }
        }

        cv_ptr->toImageMsg(res.object_proposals.overlay);

        return true;
    }



private:

    ros::NodeHandle nh_;
    pcl::PointCloud<pcl::PointXYZL > labeled_cloud_;
    std::map<uint32_t,pcl::PointCloud<pcl::PointXYZL> > cluster_map_;
    image_geometry::PinholeCameraModel model_;
    bool camera_initialized_;
    bool proposalPubEnabled_;
    bool overlayPubEnabled_;
    bool objectsPubEnabled_;


    //msgs
    message_filters::Subscriber<sensor_msgs::Image> image_sub_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> pcl_sub_;

    //pub sub
    ros::Subscriber camera_info_sub_;
    boost::shared_ptr<message_filters::Synchronizer<sync_policies::ApproximateTime<sensor_msgs::PointCloud2,sensor_msgs::Image> > > syncPtr_;
    ros::Publisher proposal_pub_;
    ros::Publisher overlay_image_pub_;
    ros::Publisher object_pub_;


    //services
    ros::ServiceServer enableProposalPubSrv_;
    ros::ServiceServer enableObjectPubSrv_;
    ros::ServiceServer enableOverlayPubSrv_;
    ros::ServiceServer doObjectProposalSrv_;
    ros::ServiceServer changeCameraInfoSrv_;

    ros::ServiceClient cnn_client_;

    int file_num_;
    //clients

};

int main(int argc, char** argv){

    ros::init(argc,argv,"object_proposal_node");
    ros::NodeHandle nh("~");
    ObjectProposal proposal(nh);

    // cv::namedWindow("Display window", cv::WINDOW_AUTOSIZE );

    ros::spin();

    ROS_INFO("Terminating object proposal node ...");

    //cv::destroyAllWindows();
    return 0;
}
