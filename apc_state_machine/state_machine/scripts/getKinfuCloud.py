import rospy
import smach
import smach_ros
import threading
from apc_msgs.srv import GetKinfuPointCloud,GetKinfuPointCloudRequest

class GetKinfuCloud(smach.State):
    def __init__(self, action='get_kinfu_cloud'):
        smach.State.__init__(self, input_keys=['next_item_to_pick'],
                output_keys=['cloud_data'], outcomes=['succeeded', 'failed'])

        # wait for the service to appear
        rospy.loginfo('Waiting for /ros_kinfu/get_point_cloud to come up ...')

        self.action_ = action
        if self.action_ == 'get_kinfu_cloud': srv_name = '/ros_kinfu/get_point_cloud'

        try:
            rospy.wait_for_service(srv_name, timeout=1)
        except:
            rospy.logerr('Service of %s not available. Restart and try again.' % srv_name)
            # rospy.signal_shutdown('')

        if self.action_ == 'get_kinfu_cloud':
            self.service = rospy.ServiceProxy(srv_name, GetKinfuPointCloud)

    # ==========================================================
    def execute(self, userdata):

        try:
            userdata['cloud_data'] = {}

            req = GetKinfuPointCloudRequest()

            res = self.service.call(req)

            userdata['cloud_data'] = {'full_cloud':res.point_cloud}
            return 'succeeded'
        except:
            print "Get Kinfu Cloud state failed!!!"
            return 'failed'
