#! /usr/bin/env python
import rospy
from sensor_msgs.msg import PointCloud2
# from std_msgs import Bool
from apc_msgs.srv import CropCloud,CropCloudRequest
class testing:
    def __init__(self):
        rospy.init_node('testToteCropper', anonymous=True)

        rospy.Subscriber("/realsense/points_aligned", PointCloud2, self.pointsCallback)
        self.srv_ = rospy.ServiceProxy('/apc_3d_vision/Crop_Tote_cloud', CropCloud)
        self.pub = rospy.Publisher('/cropped_tote_cloud', PointCloud2, queue_size=10)
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()

    def pointsCallback(self,pointcloud):
        #get the points out of the subscriber
        # split = split_labelled_point_cloud()
        # split.labelled_points = pointcloud

        req = CropCloudRequest()
        req.input_cloud = pointcloud
        req.updateTF.data = True
        req.segmentPointcloud.data = True
        print "Sending Request"
        # Response = self.srv_(req.input_cloud.data, req.updateTF.data, req.segmentPointcloud)
        Response = self.srv_(req.input_cloud, req.updateTF, req.segmentPointcloud)
        print "Got the Reponse!"
        # print Response.header
        self.pub.publish(Response.segmented_cloud)
        # print Response


if __name__ == '__main__':
    test=testing()
