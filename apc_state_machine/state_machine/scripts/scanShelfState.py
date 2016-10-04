import rospy
import smach
from apc_3d_vision.srv import shelf_scan_bin, shelf_scan_binRequest

class ScanShelfState(smach.State):
    def __init__(self, action='shelf_scan_bin'):
        smach.State.__init__(self, input_keys=['next_item_to_pick'],output_keys=['next_item_to_pick'],
                             outcomes=['succeeded', 'failed'])

        # wait for the service to appear
        rospy.loginfo('Waiting for shelf scan to come up ...')

        self.which_arm = rospy.get_param("/apc_global/which_arm","left_arm")

        self.action_ = action
        if self.action_ == 'shelf_scan_bin': srv_name = '/shelf_scan_bin'
        if self.action_ == 'shelf_scan': srv_name = '/shelf_scan'

        try:
            rospy.wait_for_service(srv_name, timeout=1)
        except:
            rospy.logerr('Service of %s not available. Restart and try again.' % srv_name)

        if self.action_ == 'shelf_scan_bin':
            self.service = rospy.ServiceProxy(srv_name, shelf_scan_bin)
        if self.action_ == 'shelf_scan':
            self.service = rospy.ServiceProxy(srv_name, shelf_scan_bin)

    # ==========================================================
    def execute(self, userdata):

        req = shelf_scan_binRequest()
        req.move_group.data = self.which_arm + "_realsense"
        req.bin_id.data = userdata['next_item_to_pick']['bin']
        req.set_scan_radius.data = True
        req.scan_radius.data = 0.05
        # req.scan_radius.data = 0.0
        if req.bin_id.data in ['bin_K','bin_F']:
            req.scan_radius.data = 0.0

        res = self.service.call(req)
        if res.success.data:
            return 'succeeded'
        else:
            return 'failed'
