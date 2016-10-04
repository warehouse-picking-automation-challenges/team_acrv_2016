#! /usr/bin/env python

import roslib
import rospy
import smach
import smach_ros

import threading

# from findObject2DState import FindObject2DState
from abortState import AbortState
from decideGraspPoseStateFromPointCloud import DecideGraspPoseStateFromPointCloud
from decideNextPickItemState import DecideNextPickItemState
from detectObjectState import DetectObjectState
from moveRobotState import MoveRobotState
from moveRobotToNamedPose import MoveRobotToNamedPose
from moveRobotToRelativePose import MoveRobotToRelativePose
from popItemState import PopItemState
from publisherState import PublisherState
from getBinCoordinatesState import GetBinCoordinatesState
from graspObjectState import GraspObjectState
from toggleBinFillersAndTote import ToggleBinFillersAndTote
from updateCollisionState import UpdateCollisionState
# import decideGraspPoseStateFromPointCloud
from suctionState import SuctionState
from getKinfuCloud import GetKinfuCloud
from shelfBasedCropCloud import ShelfBasedCropCloud
import waitState

from geometry_msgs.msg import Pose, Point, Quaternion
from baxter_core_msgs.msg import EndEffectorCommand
from std_msgs.msg import Empty

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
        # decide which object to grasp next from which bin
        sm.add('decide_next_pick', DecideNextPickItemState(),
            transitions={'succeeded':'reset_the_shelf_collision_scene_init', 'finished': 'succeeded'})

        sm.add('reset_the_shelf_collision_scene_init', ToggleBinFillersAndTote(action='fill_bins'),
            transitions={'succeeded':'set_the_shelf_collision_scene',
             'failed': 'aborted'})

        # fill unused bins with collision objects
        # actions: fill_bins, unfill_bins
        sm.add('set_the_shelf_collision_scene', ToggleBinFillersAndTote(action='fill_bins'),
            transitions={'succeeded':'reset_kinfu',
             'failed': 'aborted'})

        sm.add('reset_kinfu',
             PublisherState(topic='/ros_kinfu/reset',
                     datatype=Empty, data={}),
             transitions={'succeeded':'wait_sync1', 'aborted':'aborted'})

        sm.add('wait_sync1', waitState.WaitState(2.0),
             transitions={'succeeded':'get_kinfu_cloud'})

        sm.add('get_kinfu_cloud',GetKinfuCloud(),
             transitions={'succeeded':'crop_kinfu_cloud','failed':'aborted'})

        sm.add('crop_kinfu_cloud',ShelfBasedCropCloud(),
             transitions={'succeeded':'detect_object','failed':'aborted'})

        sm.add('detect_object', DetectObjectState(),
            transitions={'no_objects':'decide_grasp_pose',

                         'succeeded':'decide_grasp_pose',
                         'confidence_too_low':'detect_object'})  # TODO add number of trials for the same object

        sm.add('decide_grasp_pose', DecideGraspPoseStateFromPointCloud(),
            transitions={'succeeded':'detect_object',
                         'failed':'detect_object'})


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
