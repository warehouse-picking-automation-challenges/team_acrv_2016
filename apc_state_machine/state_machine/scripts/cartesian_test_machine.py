#! /usr/bin/env python

import roslib
import rospy
import smach
import smach_ros

import threading
import waitState

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
from suctionState import SuctionState


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
        sm.add('move_cart_forwards', MoveRobotToRelativePose(movegroup='left_arm_cartesian',
            pose_frame_id='/shelf',
            relative_pose=Pose(position=Point(-0.1,0,0),orientation=Quaternion(0,0,0,1)),
            velocity_scale=1.0),
                                 transitions={'succeeded':'move_cart_backwards',
                                              'failed':'move_cart_backwards'})

        sm.add('move_cart_backwards', MoveRobotToRelativePose(movegroup='left_arm_cartesian',
            pose_frame_id='/shelf',
            relative_pose=Pose(position=Point(0.1,0,0),orientation=Quaternion(0,0,0,1)),
            velocity_scale=1.0),
                                 transitions={'succeeded':'move_cart_forwards',
                                              'failed':'move_cart_forwards'})


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
