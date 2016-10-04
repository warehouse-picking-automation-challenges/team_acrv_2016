import rospy
import smach
from apc_msgs.srv import UpdateEndpointWeight, UpdateEndpointWeightRequest
from std_srvs.srv import Empty, EmptyRequest

class AddObjectWeight(smach.State):
    def __init__(self, action='add_object_weight'):
        smach.State.__init__(self, input_keys=['next_item_to_pick'],
                             outcomes=['succeeded', 'failed'])


        self.action_ = action

        if self.action_ == 'add_object_weight': srv_name = '/apc_grasping/update_endpoint_weight'
        if self.action_ == 'remove_object_weight': srv_name = '/apc_grasping/delete_endpoint_weight'

        try:
            rospy.wait_for_service(srv_name, timeout=1)
        except:
            rospy.logerr('Service of %s not available. Restart and try again.' % srv_name)

        if self.action_ == 'add_object_weight':
            self.service = rospy.ServiceProxy(srv_name, UpdateEndpointWeight)
        if self.action_ == 'remove_object_weight':
            self.service = rospy.ServiceProxy(srv_name, Empty)

    # ==========================================================
    def execute(self, userdata):

        if self.action_ == 'add_object_weight':
            req = UpdateEndpointWeightRequest()
            req.item_id = userdata['next_item_to_pick']['item']

            res = self.service.call(req)
            if res.success.data:
                return 'succeeded'
            else:
                return 'failed'

        if self.action_ == 'remove_object_weight':
            req = EmptyRequest()

            res = self.service.call(req)
            return 'succeeded'
