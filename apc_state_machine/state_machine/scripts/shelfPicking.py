
#!/usr/bin/env python

import rospy
import actionlib

from apc_msgs.msg import FindObjects2DAction
from apc_msgs.msg import FindObjects2DActionGoal

# some constants
LEFT_ARM = 1
RIGHT_ARM = 2


class ShelfPicking:

    def __init__(self):

        self.preparations()
        self.init_action_clients()

    # =========================================================================
    def preparations(self):
        rospy.loginfo('Running preparations ...')
        rospy.loginfo('  register shelf wrt robot base ...')
        rospy.loginfo('  do this ...')
        rospy.loginfo('  do that ...')
        rospy.loginfo('Done.')

    # =========================================================================
    def main_loop(self):


        # decide which object to pick from which bin
        next_object, next_bin = self.decide_next_pick()

        # move the arm to the specified bin
        status = 'trying'
        trials = 0
        while status != 'succeeded' and trials<5:
            status = self.move_arm_to_bin(bin=next_bin, arm=RIGHT_ARM)
            trials += 1

        if status == 'succeeded':
            # look for objects
            self.find_objects_2d(objects=[next_object])




    # =========================================================================
    def init_action_clients(self, wait_for_servers=True):

        rospy.loginfo('Creating action clients ...')
        self.ac={}
        self.ac['find_object_2d'] = actionlib.SimpleActionClient('find_object_2d_wrapper', FindObjects2DAction)


        if wait_for_servers:
            rospy.loginfo('Waiting for action servers to come online ...')
            for c in self.ac:
                rospy.loginfo('  ' + c)
                self.ac[c].wait_for_server()

        rospy.loginfo('Done.')


    # =========================================================================
    def find_objects_2d(self, objects=[], timeout=10.0):

        # fill the goal message (not implemented yet on the other side)
        goal = FindObjects2DActionGoal()
        goal.goal.ids.data = objects

        # start the action
        rospy.loginfo('Looking for objects in 2D ...')
        ac = self.ac['find_object_2d']
        ac.send_goal(goal.goal)

        # wait for results, with timeout
        ac.wait_for_result(timeout=rospy.Duration(timeout))
        result = ac.get_result()


        # process the results
        if result is None:
            rospy.logwarn('No 2D object models found.')
            return []
        elif len(result.objects.data) == 0:
            rospy.logwarn('No 2D object models found.')
            return []
        else:
            data = result.objects.data
            found_objects = [int(data[i]) for i in range(0,len(data),12)]
            print 'found objects:', found_objects
            return found_objects




# =============================================================================
# =============================================================================
# =============================================================================
if __name__ == '__main__':


    rospy.init_node('shelf_picking')

    shelfPicking = ShelfPicking()

    shelfPicking.find_objects_2d([3,6])
