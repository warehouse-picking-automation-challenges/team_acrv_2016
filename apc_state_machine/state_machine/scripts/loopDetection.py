#! /usr/bin/env python

import roslib
import rospy
import smach
import smach_ros

import threading

# from findObject2DState import FindObject2DState
from detectObjectState import DetectObjectState
from decideGraspPoseState import DecideGraspPoseState
from moveRobotState import MoveRobotState
from decideNextPickItemState import DecideNextPickItemState
from popItemState import PopItemState
from getBinCoordinatesState import GetBinCoordinatesState
from publisherState import PublisherState
from decideGraspPoseStateFromPointCloud import DecideGraspPoseStateFromPointCloud
import waitState

from geometry_msgs.msg import Pose, Point, Quaternion
from baxter_core_msgs.msg import EndEffectorCommand

# =============================================================================
# Documentation fields in the user_data of the state machine:
# next_item_to_pick :   {'bin': a string encoding from which bin to pick, 'item': a string with the item name}
# goal_frame_id :       a string with the TF frame of where to move the arm, used by MoveRobotState
# goal_pose :           a 3D pose in the goal_frame_id frame, describing where to move the arm, used by MoveRobotState

# =============================================================================
# =============================================================================
# =============================================================================
if __name__ == '__main__':
    rospy.init_node('smach_shelf_reach_testing')

    # Create the top level SMACH state machine
    sm = smach.StateMachine(outcomes=['succeeded', 'aborted', 'preempted'])

    # create states and connect them
    with sm:

        sm.add('decide_next_pick', DecideNextPickItemState(),
            transitions={'succeeded':'get_target_bin_coordinates', 'finished': 'succeeded'})

        sm.add('get_target_bin_coordinates', GetBinCoordinatesState(),
            transitions={'succeeded':'detect_object','aborted':'detect_object'})

        sm.add('detect_object', DetectObjectState(),
            transitions={'no_objects':'decide_next_pick',
                         'succeeded':'decide_grasp_pose'})

        sm.add('decide_grasp_pose', DecideGraspPoseStateFromPointCloud(),
            transitions={'succeeded':'decide_grasp_pose', 'failed':'decide_grasp_pose'})


    # Create and start the introspection server
    #  (smach_viewer is broken in indigo + 14.04, so need for that now)
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    # run the state machine
    #   We start it in a separate thread so we can cleanly shut it down via CTRL-C
    #   by requesting preemption.
    #   The state machine normally runs until a terminal state is reached.
    smach_thread = threading.Thread(target=sm.execute)
    smach_thread.start()
    # sm.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()

    # request the state machine to preempt and wait for the SM thread to finish
    sm.request_preempt()
    smach_thread.join()

    # stop the introspection server
    sis.stop()
