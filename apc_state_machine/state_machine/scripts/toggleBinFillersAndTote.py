import rospy
import smach
import smach_ros
import threading
import tf
from tf import transformations
import math
import yaml
from std_srvs.srv import Trigger, TriggerRequest,SetBool,SetBoolRequest
from apc_msgs.srv import FillUnfillBinsCollisionModel, FillUnfillBinsCollisionModelRequest

class ToggleBinFillersAndTote(smach.State):
    def __init__(self, action='fill_bins'):
        smach.State.__init__(self, input_keys=['next_item_to_pick'],
                             outcomes=['succeeded', 'failed'])

        # wait for the service to appear
        rospy.loginfo('Waiting for fill/unfill bins and toggle tote to come up ...')

        self.action_ = action
        if self.action_ == 'fill_bins' or self.action_ == 'fill_bins_tote': srv_name = '/apc_grasping/fill_bins'
        if self.action_ == 'unfill_bins': srv_name = '/apc_grasping/unfill_bins'
        if self.action_ == 'tote_on' or self.action_ == 'pillar_on' or self.action_ == 'tote_off' or self.action_ == 'pillar_off' or self.action_ == 'all_on' or self.action_ == 'all_off':
            srv_name = '/apc_grasping/move_tote'
        if self.action_ == 'lip_on' or self.action_ == 'lip_off': srv_name = '/apc_grasping/toggle_lip'

        try:
            rospy.wait_for_service(srv_name, timeout=1)
        except:
            rospy.logerr('Service of %s not available. Restart and try again.' % srv_name)
            # rospy.signal_shutdown('')

        if self.action_ == 'fill_bins' or self.action_ == 'fill_bins_tote':
            self.service = rospy.ServiceProxy(srv_name, FillUnfillBinsCollisionModel)
        if self.action_ == 'unfill_bins':
            self.service = rospy.ServiceProxy(srv_name, Trigger)
        if self.action_ == 'tote_on' or self.action_ == 'pillar_on' or self.action_ == 'tote_off' or self.action_ == 'pillar_off' or self.action_ == 'all_on' or self.action_ == 'all_off':
            self.service = rospy.ServiceProxy(srv_name, FillUnfillBinsCollisionModel)
        if self.action_ == 'lip_on' or self.action_ == 'lip_off':
            self.service = rospy.ServiceProxy(srv_name, SetBool)

    # ==========================================================
    def execute(self, userdata):
        if self.action_ == 'fill_bins' or self.action_ == 'fill_bins_tote': return self.execute_fill_bins(userdata)
        if self.action_ == 'unfill_bins': return self.execute_unfill_bins(userdata)
        if self.action_ == 'tote_on' or self.action_ == 'pillar_on' or self.action_ == 'tote_off' or self.action_ == 'pillar_off' or self.action_ == 'all_on' or self.action_ == 'all_off': return self.execute_toggle_tote(userdata)
        if self.action_ == 'lip_on' or self.action_ == 'lip_off': return self.execute_toggle_lip()

    def execute_fill_bins(self, userdata):
        # try to call the service

        fillMsg = FillUnfillBinsCollisionModel()
        print userdata['next_item_to_pick']['bin']
        try:
            bin_id = userdata['next_item_to_pick']['bin']
            if self.action_ == 'fill_bins_tote':
                bin_id = 'bin_A'
            req = FillUnfillBinsCollisionModelRequest()
            req.bin_id.data = bin_id
            response = self.service.call(req)
            rospy.loginfo('All bins except %s hav been filled!' % bin_id)
            return 'succeeded'
        except Exception,e:
            rospy.logerr('Service call to fill_bins failed. Reason: \n %s' % e)
            return 'failed'

    def execute_unfill_bins(self, userdata):
        # try to call the service
        try:
            req = TriggerRequest()
            response = self.service.call(req)
            rospy.loginfo('All bins have been unfilled')
            return 'succeeded'
        except Exception,e:
            rospy.logerr('Service call to unfill_bins failed. Reason: \n %s' % e)
            return 'failed'

    def execute_toggle_tote(self, userdata):
        # try to call the service
        req = FillUnfillBinsCollisionModelRequest()
        req.bin_id.data = self.action_
        try:
            response = self.service.call(req)
            rospy.loginfo('Tote has been toggled on/off')
            return 'succeeded'
        except Exception,e:
            rospy.logerr('Service call to move_tote failed. Reason: \n %s' % e)
            return 'failed'

    def execute_toggle_lip(self):
        # try to call the service
        req = SetBoolRequest()
        if self.action_ is 'lip_on':
            req.data = True
        elif self.action_ is 'lip_off':
            req.data = False

        try:
            response = self.service.call(req)
            rospy.loginfo('Lip has been toggled on/off')
            return 'succeeded'
        except Exception,e:
            rospy.logerr('Service call to toggle_lip failed. Reason: \n %s' % e)
            return 'failed'
