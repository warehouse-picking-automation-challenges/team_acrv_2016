import rospy
import smach
import smach_ros
import threading
import time
from apc_msgs.srv import DoSegmentation,DoSegmentationRequest,FillUnfillBinsCollisionModel,FillUnfillBinsCollisionModelRequest
from sensor_msgs.msg import Image

################################################################################
#
# NOTE: Image captured here as the shelf cropping service returns a transformed
# point cloud (instead of kf_world) with respect to camera_rgb_optical_frame and
# image should be captured at this time
#
################################################################################

class ShelfBasedCropCloud(smach.State):
    def __init__(self, action='crop_cloud'):
        smach.State.__init__(self, input_keys=['cloud_data','next_item_to_pick'],
                output_keys=['cloud_data'], outcomes=['succeeded', 'failed'])

        # wait for the service to appear
        rospy.loginfo('Waiting for /ros_kinfu/get_point_cloud to come up ...')

        self.image = None
        self.got_image = False

        change_bin_srv_name = '/apc_3d_vision/split_cloud_change_bin'
        self.action_ = action

        srv_name = '/apc_3d_vision/split_cloud'

        try:
            rospy.wait_for_service(srv_name, timeout=1)
            rospy.wait_for_service(change_bin_srv_name, timeout=1)
        except:
            rospy.logerr('Service of %s not available. Restart and try again.' % srv_name)
            # rospy.signal_shutdown('')
        if self.action_ == 'crop_kinfu_cloud':
            self.image_sub = rospy.Subscriber('/kinect2/qhd/image_color',Image,self.image_callback)
        else:
            self.image_sub = rospy.Subscriber('/realsense/rgb/image_raw',Image,self.image_callback)

        self.service = rospy.ServiceProxy(srv_name, DoSegmentation)
        self.change_bin_service = rospy.ServiceProxy(change_bin_srv_name, FillUnfillBinsCollisionModel)


    def image_callback(self,image):
        if not self.got_image:
            self.image = image

    # ==========================================================
    def execute(self, userdata):

        try:
            req = FillUnfillBinsCollisionModelRequest()
            print self.action_
            if self.action_ == 'crop_cloud':
                req.bin_id.data = userdata['next_item_to_pick']['bin']
            if self.action_ == 'crop_tote':
                req.bin_id.data = 'tote'
            if self.action_ == 'crop_kinfu_cloud':
                req.bin_id.data = 'kinect'

            res = self.change_bin_service.call(req)
            print "Change bin service response = ", res
            print self.action_

            req = DoSegmentationRequest()
            req.input_cloud = userdata['cloud_data']['full_cloud']
            if self.action_ == 'crop_cloud':
                req.input_cloud.header.frame_id = 'kf_world'                    ### BAD, why isn't this set?
            if self.action_ == 'crop_tote':
                req.input_cloud.header.frame_id = 'kf_world'                    ### BAD, why isn't this set?
            if self.action_ == 'crop_kinfu_cloud':
                req.input_cloud.header.frame_id = 'kinect2_rgb_optical_frame'

            while self.image is None:
                rospy.loginfo('Waiting for image ...')
                time.sleep(0.15)

            self.got_image = True

            print "Seg cloud height: ", req.input_cloud.height, "\nSeg cloud width: ", req.input_cloud.width
            res = self.service.call(req)

            # print "Crop shelf service response = ", res

            # if res.success.data:
            userdata['cloud_data']['cropped_cloud'] = res.segmented_cloud
            userdata['cloud_data']['image'] = self.image
            self.got_image = False
            return 'succeeded'
        except Exception as e:
            print e
            print "Shelf crop state failed!!!!"
            self.got_image = False
            return 'failed'
