import rospy
import smach
import smach_ros
import mission_plan.srv

from std_msgs.msg import String
from apc_msgs.srv import AttachObjectCollisionModel, AttachObjectCollisionModelRequest
from apc_msgs.srv import DetachObjectCollisionModel, DetachObjectCollisionModelRequest


class UpdateCollisionState(smach.State):
    def __init__(self, action='attach'):
        smach.State.__init__(self, input_keys=['next_item_to_pick', 'move_group'],
                             output_keys=['next_item_to_pick'],
                             outcomes=['succeeded', 'aborted'])

        # wait for the service to appear
        rospy.loginfo('Waiting for services of attach/detach collision model to come up ...')

        self.action_ = action
        if self.action_ == 'attach': srv_name = '/apc_grasping/attach_object_collision_model'
        if self.action_ == 'detach': srv_name = '/apc_grasping/detach_object_collision_model'
        try:
            rospy.wait_for_service(srv_name, timeout=1)
        except:
            rospy.logerr('Service of %s not available. Restart and try again.' % srv_name)
            # rospy.signal_shutdown('')

        self.get_item_id_from_name_service = rospy.ServiceProxy('/mission_plan/get_item_id_from_name', mission_plan.srv.GetItemIDFromName)

        if self.action_ == 'attach':
            self.service = rospy.ServiceProxy(srv_name, AttachObjectCollisionModel)
        if self.action_ == 'detach':
            self.service = rospy.ServiceProxy(srv_name, DetachObjectCollisionModel)

    # ==========================================================
    def execute(self, userdata):
        # Ensure the item id is always filled in. Using the id is legacy as we have moved to using the item name only
        string_msg = String()
        string_msg.data = userdata['next_item_to_pick']['item']
        response = self.get_item_id_from_name_service.call(string_msg)
        userdata['next_item_to_pick']['id'] = response.id.data

        if self.action_ == 'attach': return self.execute_attach(userdata)
        if self.action_ == 'detach': return self.execute_detach(userdata)

    def execute_attach(self, userdata):
        # try to call the service
        try:
            data = userdata['next_item_to_pick']
            req = AttachObjectCollisionModelRequest()
            req.object_id.data = data['id'] # = {'bin':response.bin.data, 'item':response.item.data, 'id':response.id.data}
            req.radius.data = 0.07

            if userdata['move_group'] == 'left_arm' or userdata['move_group'] == 'left_arm_cartesian':
                req.link_to_attach_to.data = 'left_gripper'
            if userdata['move_group'] == 'left_arm_90' or userdata['move_group'] == 'left_arm_90_cartesian':
                req.link_to_attach_to.data = 'left_gripper_90'

            if userdata['move_group'] == 'right_arm' or userdata['move_group'] == 'right_arm_cartesian':
                req.link_to_attach_to.data = 'right_gripper'
            if userdata['move_group'] == 'right_arm_90' or userdata['move_group'] == 'right_arm_90_cartesian':
                req.link_to_attach_to.data = 'right_gripper_90'

            response = self.service.call(req)
            # print response
            rospy.loginfo('Object %s has been attached!' % data['item'])
            return 'succeeded'
        except Exception,e:
            rospy.logerr('Service call to attach_object failed. Reason: \n %s' % e)
            return 'aborted'

    def execute_detach(self, userdata):
        # try to call the service
        try:
            data = userdata['next_item_to_pick']
            req = DetachObjectCollisionModelRequest()
            req.object_id.data = data['id'] # = {'bin':response.bin.data, 'item':response.item.data, 'id':response.id.data}

            if userdata['move_group'] == 'left_arm' or userdata['move_group'] == 'left_arm_cartesian':
                req.link_it_is_attached_to.data = 'left_gripper'
            if userdata['move_group'] == 'left_arm_90' or userdata['move_group'] == 'left_arm_90_cartesian':
                req.link_it_is_attached_to.data = 'left_gripper_90'

            if userdata['move_group'] == 'right_arm' or userdata['move_group'] == 'right_arm_cartesian':
                req.link_it_is_attached_to.data = 'right_gripper'
            if userdata['move_group'] == 'right_arm_90' or userdata['move_group'] == 'right_arm_90_cartesian':
                req.link_it_is_attached_to.data = 'right_gripper_90'

            response = self.service.call(req)
            # print response
            rospy.loginfo('Object %s has been detached!' % data['item'])
            return 'succeeded'
        except Exception,e:
            rospy.logerr('Service call to detach_object failed. Reason: \n %s' % e)
            return 'aborted'
