import rospy
from sensor_msgs.msg import Image
import smach
import time
import subprocess
from cv_bridge import CvBridge, CvBridgeError
import cv2
from apc_msgs.srv import DoSegmentation,DoSegmentationRequest,DoObjectProposal,DoObjectProposalRequest
from apc_msgs.msg import PointcloudRGB


class GetData(smach.State):
    def __init__(self, action='nothing'):
        smach.State.__init__(self,input_keys=['cropped_cloud'],
                             outcomes=['succeeded', 'failed'])
        self.boolCallback = False
        self.image_count = 399
        self.bridge = CvBridge()
        self.counter = 399
        self.dir_save_bagfile = '/home/apc/co/apc_ws/DataBase/ROSBAG22June/'
        # rospy.init_node('GetDataImage')
        rospy.Subscriber("/realsense/rgb/image_raw",Image,self.callback)
        # rospy.spin()
        self.waitForService('/object_proposal_node/do_proposal')
        self.waitForService('/segmentation_node/do_segmentation')

        self.srv_proposal = rospy.ServiceProxy('/object_proposal_node/do_proposal', DoObjectProposal)
        self.srv_segmentation = rospy.ServiceProxy('/segmentation_node/do_segmentation', DoSegmentation)


    # ==========================================================

# ==========================================================
    def waitForService(self, service):

        rospy.loginfo('Waiting for service %s to come online ...' % service)
        try:
            rospy.wait_for_service(service, timeout=0.1)
        except:
            rospy.logerr('Service %s not available. Restart and try again.' % service)
            return False
        else:
            return True

    def execute(self, userdata):
        # if (self.counter<30):
        try:
            command = "rosbag record /tf /realsense/rgb/image_raw /realsense/points_aligned /realsense/rgb/camera_info /realsense/depth_aligned/camera_info /realsense/depth_aligned/image_raw /realsense/points --duration 3"

            req = DoSegmentationRequest()
            req.input_cloud = userdata['cropped_cloud']
            segmented_cloud = self.srv_segmentation.call(req).segmented_cloud

        # form a PointcloudRGB message rofor the proposal service
            pointCloudRGB = PointcloudRGB(segmented_cloud, self.current_rgb)

        # call the DoProposal service, receive a ObjectProposals instance in the response
            rospy.loginfo('Calling object proposal service ... ')
            proposals = self.srv_proposal.call(pointCloudRGB).object_proposals
            text_file = open('/home/apc/co/apc_ws/DataBase/IMAGES22June/Image'+str(self.image_count).zfill(4)+'.txt', "w")
            for bb in proposals.bounding_boxes:
                text_file.write("{} {} {} {}\n".format(str(bb.top_left.x), str(bb.top_left.y), str(bb.bottom_right.x), str(bb.bottom_right.y)))
            mask_count = 0
            for mask in proposals.masks:
                try:
                    print "test"
                    cv_image = self.bridge.imgmsg_to_cv2(mask, "bgr8")
                    cv2.imwrite('/home/apc/co/apc_ws/DataBase/IMAGES22June/Image'+str(self.image_count).zfill(4)+'_mask'+str(mask_count).zfill(4)+'.jpeg',cv_image)
                except CvBridgeError as e:
                    print e
                mask_count = mask_count + 1
            text_file.close()
            self.boolCallback = True
            p = subprocess.Popen(command, stdin=subprocess.PIPE, shell=True, cwd=self.dir_save_bagfile)
            time.sleep(1)

            self.counter = self.counter+1
            print self.counter

            return 'succeeded'
        except:
            return 'failed'


    def callback(self,data):
        self.current_rgb = data
        if self.boolCallback:
            self.boolCallback = False
            #re
            try:
                cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            except CvBridgeError as e:
                print e

            cv2.imwrite('/home/apc/co/apc_ws/DataBase/IMAGES22June/Image'+str(self.image_count).zfill(4)+'.jpeg',cv_image)
            print "FINISHED image storage"
            self.image_count = self.image_count+1
