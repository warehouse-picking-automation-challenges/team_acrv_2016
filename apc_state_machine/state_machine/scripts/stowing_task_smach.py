#! /usr/bin/env python

import roslib
import rospy
import smach
import smach_ros

import threading

# from findObject2DState import FindObject2DState
from detectObjectState import DetectObjectState
from moveRobotState import MoveRobotState
from moveRobotToNamedPose import MoveRobotToNamedPose
from moveRobotToRelativePose import MoveRobotToRelativePose
from decideNextPickItemState import DecideNextPickItemState
from popItemState import PopItemState
from stowSuccessfulState import StowSuccessfulState
from getBinCoordinatesState import GetBinCoordinatesState
from publisherState import PublisherState
from decideGraspPoseStateFromPointCloud import DecideGraspPoseStateFromPointCloud
from decideGraspPoseState import DecideGraspPoseState
from graspObjectState import GraspObjectState
from updateCollisionState import UpdateCollisionState
from toggleBinFillersAndTote import ToggleBinFillersAndTote
from suctionState import SuctionState
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

    relative_movement = 0.33
    # create states and connect them
    with sm:
        # decide which object to grasp next from which bin
        sm.add('get_next_item', DecideNextPickItemState(),
            transitions={'succeeded':'set_the_shelf_collision_scene', 'finished': 'succeeded'})
            # transitions={'succeeded':'add_object_collision', 'finished': 'succeeded'})

        sm.add('set_the_shelf_collision_scene', ToggleBinFillersAndTote(action='fill_bins'),
            transitions={'succeeded':'set_the_tote_collision_scene',
             'failed': 'abort_state'})

        # add tote and tote pillar to collision scene
        # tote/pillar actions: tote_on, tote_off, pillar_on, pillar_off, all_on, all_off
        sm.add('set_the_tote_collision_scene', ToggleBinFillersAndTote(action='all_on'),
            transitions={'succeeded':'move_to_tote',
             'failed': 'abort_state'})


        # move your hand above the tote to look into it
        sm.add('move_to_tote', MoveRobotToNamedPose(movegroup='left_arm',
                                    goal_pose_name='look_into_tote'),
            # transitions={'succeeded':'move_arm_to_object',
            transitions={'succeeded':'detect_object',
                         'failed': 'abort_state'})

        sm.add('detect_object', DetectObjectState(),
            transitions={'no_objects':'detect_object',
                         'succeeded':'turn_tote_collision_off',
                         'confidence_too_low':'detect_object'})  # TODO add number of trials for the same object
                        #  'succeeded':'move_arm_infront_of_object'})


        sm.add('turn_tote_collision_off', ToggleBinFillersAndTote(action='tote_off'),
            transitions={'succeeded':'decide_grasp_pose',
             'failed': 'abort_state'})

        # perform PCL segmenation and pick a good object / point to grasp
        sm.add('decide_grasp_pose', DecideGraspPoseStateFromPointCloud(),
            transitions={'succeeded':'move_arm_to_object', 'failed':'decide_grasp_pose'})


        # move to object tf
        sm.add('move_arm_to_object', GraspObjectState(movegroup='left_arm', velocity_scale=0.5),
            transitions={'succeeded':'lift_object_up',
                         'failed':'deactivate_suction_and_abort',  # Infinite loop, state must say aborted. Don't think this happens yet
                         'aborted':'deactivate_suction_and_abort'})

        # Lift up the object
        # move your hand back out of the tote
        sm.add('lift_object_up', MoveRobotToNamedPose(movegroup='left_arm',
                                        goal_pose_name='left_tote'),
            transitions={'succeeded':'add_object_collision',
            # transitions={'succeeded':'get_target_bin_coordinates',
                         'failed': 'abort_state'})

        sm.add('add_object_collision', UpdateCollisionState(action='attach'),
            transitions={'succeeded':'get_target_bin_coordinates'})

        # calculate which bin the object has to go to
        sm.add('get_target_bin_coordinates', GetBinCoordinatesState(),
            transitions={'succeeded':'move_arm_to_bin','aborted':'abort_state'})

        # move the sucker into the object, colliding with it slightly\
        # still problems with the planner in here
        # move arm in front of bin before looking for the object

        sm.add('move_arm_to_bin', MoveRobotState(movegroup='left_arm'),
            transitions={'succeeded':'turn_off_the_shelf_collision_scene', 'failed':'abort_state'})

        sm.add('turn_off_the_shelf_collision_scene', ToggleBinFillersAndTote(action='unfill_bins'),
            transitions={'succeeded':'move_sucker_in',
             'failed': 'abort_state'})
        # TODO probably would need to look into the bin to see where in the bin there is space :)

        # move the object into the shelf bin
        sm.add('move_sucker_in', MoveRobotToRelativePose(movegroup='left_arm',
                                        pose_frame_id='/shelf',
                                        #  TODO fix that based on the posiiton we are at!
                                        relative_pose=Pose(position=Point(0,0,relative_movement),
                                                       orientation=Quaternion(0,0,0,1)),
                                        velocity_scale=0.5),
             transitions={'succeeded':'deactivate_suction',
                          'failed':'abort_state'})

        # deactivate suction to let go of the object
        sm.add('deactivate_suction', SuctionState(state='off', movegroup=None),
            transitions={'succeeded':'remove_object_collision', 'failed':'abort_state'})

        sm.add('remove_object_collision', UpdateCollisionState(action='detach'),
            transitions={'succeeded':'move_sucker_back_out'})

        # sm.add('remove_current_pick', PopItemState(),
        #     transitions={'succeeded':'move_to_neutral'})

        sm.add('move_sucker_back_out', MoveRobotToRelativePose(
                                            movegroup='left_arm',
                                            pose_frame_id='/shelf',
                                            relative_pose=Pose(position=Point(0,0,-relative_movement),
                                                    orientation=Quaternion(0,0,0,1)),
                                            velocity_scale=0.5),
             transitions={'succeeded':'get_next_item',
                          'failed':'abort_state'})

        # something went wrong after we had the suction on (maybe
        # there should be a field for this in user_data
        #  e.g. suction_status = 'on' or 'off' which should be checked)
        sm.add('deactivate_suction_and_abort',
            PublisherState(topic='/robot/end_effector/right_gripper/command',
                    datatype=EndEffectorCommand,
                    data={'command':'stop', 'sender':'user', 'id': 65537},
                    n=5, hz=10),
            transitions={'succeeded':'abort_state'})

        sm.add('abort_state', PopItemState(),
            transitions={'succeeded':'succeeded'})

        # sm.add('move_to_neutral', MoveRobotState(movegroup='left_arm',
        #         named=True, goal_frame_id='left_neutral'),
        #     transitions={'succeeded':'decide_next_pick',
        #                  'failed': 'abort_state'})


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
    #sis.stop()
