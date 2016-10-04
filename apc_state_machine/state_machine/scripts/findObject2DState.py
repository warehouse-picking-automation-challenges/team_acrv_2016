import rospy
import smach
import smach_ros
import threading

import actionlib
from apc_msgs.msg import FindObjects2DAction
from apc_msgs.msg import FindObjects2DActionGoal


# ==========================================================
class FindObject2DState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'no_objects', 'preempted'],
            input_keys=['sought_2d_objects'],
            output_keys=['found_2d_objects']
            )

        self.ac = actionlib.SimpleActionClient('find_object_2d_wrapper', FindObjects2DAction)

        rospy.loginfo('Waiting for action server for find_object_2d_wrapper to come online ...')
        if not self.ac.wait_for_server(timeout=rospy.Duration(2)):
            rospy.logerr('Action server for find_object_2d_wrapper not available. Restart and try again.')

    # ==========================================================
    def execute(self, userdata):

        # fill the goal message (not implemented yet on the other side)
        goal = FindObjects2DActionGoal()

        # translate from userdata[next_item_to_pick]['item'] to id
        # TODO convert object name (string) to id
        goal.goal.ids.data = [ 2000 ]  #id   #userdata['sought_objects']

        # start the action
        rospy.loginfo('Looking for objects in 2D ...')
        self.ac.send_goal(goal.goal)

        # wait for results, with timeout
        self.ac.wait_for_result(timeout=rospy.Duration(5))
        result = self.ac.get_result()

        print result
        print type(result)


        # process the results
        if result is None:
            rospy.logwarn('No 2D object models found. Result is None')
            userdata['found_2d_objects'] = []
            return 'no_objects'
        elif len(result.objects.data) == 0:
            rospy.logwarn('No 2D object models found.')
            userdata['found_2d_objects'] = []
            return 'no_objects'
        else:
            data = result.objects.data

            # retrieve found object views / face IDs from the action
            found_objects = [int(data[i]) for i in range(0,len(data),12)]
            userdata['found_2d_objects'] = found_objects
            print 'found objects:', found_objects
            return 'succeeded'
