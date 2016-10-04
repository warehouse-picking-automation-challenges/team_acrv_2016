import rospy
import smach

from baxter_core_msgs.msg import EndEffectorCommand


class SuctionState(smach.State):
    def __init__(self, state=None, movegroup=None):
        smach.State.__init__(self, outcomes=['succeeded','failed'],
                             input_keys=['move_group'])

        self.movegroup = movegroup
        self.state = state
        self.pumpPublisher = rospy.Publisher('/robot/end_effector/right_gripper/command',
                                EndEffectorCommand, queue_size = 1)
        # self.pumpPublisher = rospy.Publisher('/robot/end_effector/right_gripper/command', EndEffectorCommand, queue_size = 10)

    # ==========================================================
    def execute(self, userdata):
        # Take passed in input over userdata!
        # Only if we were not passed in a movegroup through the constructor
        # do we use the userdata
        if self.movegroup is None:
            if 'move_group' in userdata:
                if userdata['move_group'] is not None:
                    self.movegroup = userdata['move_group']
                else:
                    print 'move_group is None!'
                    return 'failed'
            else:
                rospy.logerr('no move_group in userdata!')
                return 'failed'

        if self.state == 'on':
            self.activatePump()

        elif self.state == 'off':
            self.deactivatePump()

        else:
            print 'Invalid suction state: ', self.state
            return 'failed'


        return 'succeeded'



    def activatePump(self):
        # NOTE Need to trigger line low before high
        self.deactivatePump()
        msg = EndEffectorCommand()
        msg.command = 'go'
        msg.id = 65537
        msg.sender = 'user'
        msg.args = "{\"grip_attempt_seconds\": 1000}"
        self.pumpPublisher.publish(msg)


    def deactivatePump(self):
        print "pump stop"
        msg = EndEffectorCommand()
        msg.command = 'stop'
        msg.id = 65537
        msg.sender = 'user'
        msg.args = "{\"grip_attempt_seconds\": 1000}"
        self.pumpPublisher.publish(msg)
