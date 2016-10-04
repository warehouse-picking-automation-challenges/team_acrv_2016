
import roslib
import rospy
import smach
import smach_ros


from apc_msgs.msg import FindObjects2DAction
from actionlib_msgs.msg import GoalStatus


class ObjectDetection2D(smach_ros.SimpleActionState):
    def __init__(self, goal):
        smach_ros.SimpleActionState.__init__(self,
            action_name='find_object_2d_wrapper',
            action_spec=FindObjects2DAction,
            result_cb=self.result_cb,
            goal=goal.goal,
            outcomes=['succeeded', 'preempted', 'no_objects', 'failed']
            )

    def result_cb(self, userdata, status, result):        
        if status == GoalStatus.PREEMPTED:
            return 'preempted'
        
        elif status == GoalStatus.SUCCEEDED:
            if len(result.objects.data)==0:
                return 'no_objects'
            else:
                return 'succeeded'
        else:
            return 'failed'

