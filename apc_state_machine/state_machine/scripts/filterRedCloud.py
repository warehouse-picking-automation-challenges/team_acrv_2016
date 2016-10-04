import rospy
import smach
from apc_msgs.srv import CropCloud,CropCloudRequest

################################################################################
#
# NOTE: Image captured here as the shelf cropping service returns a transformed
# point cloud (instead of kf_world) with respect to camera_rgb_optical_frame and
# image should be captured at this time
#
################################################################################

class FilterRedCloudUpdateToteTF(smach.State):
    def __init__(self, action='filter_red_cloud'):
        smach.State.__init__(self, input_keys=['cloud_data'],
                output_keys=['cloud_data'], outcomes=['succeeded', 'failed'])

        self.action_ = action

        srv_name = '/apc_3d_vision/Crop_Tote_cloud'

        try:
            print 'waiting for red cloud filter service'
            rospy.wait_for_service(srv_name, timeout=1)
        except:
            rospy.logerr('Service of %s not available. Restart and try again.' % srv_name)

        self.service = rospy.ServiceProxy(srv_name, CropCloud)

    # ==========================================================
    def execute(self, userdata):

        try:
            req = CropCloudRequest()
            if self.action_ == 'filter_red_cloud':
                req.input_cloud = userdata['cloud_data']['full_cloud']
                req.updateTF.data = True
                req.segmentPointcloud.data = True

            res = self.service.call(req)
            if res.success.data:
                userdata['cloud_data']['full_cloud'] = res.segmented_cloud
                return 'succeeded'
            else:
                print "red cloud filter returned false"
                return 'failed'
        except:
            print "FilterRedCloud state exception!!!!"
            return 'failed'
