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
from moveRobotToNamedPose import MoveRobotToNamedPose
from moveRobotToRelativePose import MoveRobotToRelativePose
from decideNextPickItemState import DecideNextPickItemState
from popItemState import PopItemState
from getBinCoordinatesState import GetBinCoordinatesState
from publisherState import PublisherState
from decideGraspPoseStateFromPointCloud import DecideGraspPoseStateFromPointCloud
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

    # create states and connect them
    with sm:
        # decide which object to grasp next from which bin
        sm.add('deactivate_suction_init',
            PublisherState(topic='/robot/end_effector/right_gripper/command',
                    datatype=EndEffectorCommand,
                    data={'command':'stop', 'sender':'user', 'id': 65537},
                    n=5, hz=10),
            transitions={'succeeded':'decide_next_pick'})
        # sm.add('activate_suction_init',
        #     PublisherState(topic='/robot/end_effector/right_gripper/command',
        #             datatype=EndEffectorCommand,
        #             data={'command':'go', 'sender':'user', 'id': 65537, 'args':'{\"grip_attempt_seconds\": 10000}'},
        #             n=5, hz=10),
        #     transitions={'succeeded':'move_arm_to_object', 'aborted':'abort_state'})



        sm.add('decide_next_pick', DecideNextPickItemState(),
            transitions={'succeeded':'reset_the_shelf_collision_scene_init', 'finished': 'succeeded'})

        sm.add('reset_the_shelf_collision_scene_init', ToggleBinFillersAndTote(action='fill_bins'),
            transitions={'succeeded':'move_to_neutral_start',
             'failed': 'abort_state'})

        sm.add('move_to_neutral_start', MoveRobotToNamedPose(movegroup='left_arm',
                                    goal_pose_name='left_neutral'),
                transitions={'succeeded':'remove_object_collision',    # 'lift_object_up'
                             'failed': 'abort_state'})

        sm.add('remove_object_collision', UpdateCollisionState(action='detach'),
            transitions={'succeeded':'set_the_shelf_collision_scene'})

       # fill unused bins with collision objects
       # actions: fill_bins, unfill_bins
        sm.add('set_the_shelf_collision_scene', ToggleBinFillersAndTote(action='fill_bins'),
            transitions={'succeeded':'set_the_tote_collision_scene',
             'failed': 'abort_state'})

        # add tote and tote pillar to collision scene
        # tote/pillar actions: tote_on, tote_off, pillar_on, pillar_off, all_on, all_off
        sm.add('set_the_tote_collision_scene', ToggleBinFillersAndTote(action='all_on'),
            transitions={'succeeded':'get_target_bin_coordinates',
             'failed': 'abort_state'})

        # move arm in front of bin before looking for the object
        sm.add('get_target_bin_coordinates', GetBinCoordinatesState(),
            transitions={'succeeded':'move_arm_to_look_into_bin','aborted':'abort_state'})

        # sm.add('move_arm_to_look_into_bin', MoveRobotState(movegroup='left_arm_realsense'),
        sm.add('move_arm_to_look_into_bin', MoveRobotToNamedPose(movegroup='left_arm',
                                    name_prefix='look_into_'),
            transitions={'succeeded':'detect_object',
                         'failed': 'abort_state',
                         'aborted':'abort_state'})
            # transitions={'succeeded':'remove_current_pick', 'aborted':'abort_state'})

        sm.add('wait_sync1', waitState.WaitState(0.2),
            transitions={'succeeded':'detect_object'})

        sm.add('detect_object', DetectObjectState(),
            transitions={'no_objects':'decide_next_pick',
                         'succeeded':'move_sucker_infront_of_bin',
                         'confidence_too_low':'detect_object'})  # TODO add number of trials for the same object
                        #  'succeeded':'move_arm_infront_of_object'})

        sm.add('decide_grasp_pose', DecideGraspPoseStateFromPointCloud(),
            transitions={'succeeded':'move_sucker_infront_of_bin', 'failed':'detect_object'})

        # sm.add('decide_grasp_pose_from_2d', DecideGraspPoseState(),
        #     transitions={'succeeded':'move_sucker_infront_of_bin', 'failed':'remove_current_pick'})

        # move the sucker into the object, colliding with it slightly\
        # still problems with the planner in here
        sm.add('move_sucker_infront_of_bin', MoveRobotToNamedPose(movegroup='left_arm',
                        name_prefix='ready_to_suck_in_'),
            transitions={'succeeded':'move_arm_to_object',
                         'failed': 'abort_state',
                         'aborted':'abort_state'})


        # activate suction here before moving the arm
        # sm.add('activate_suction',
        #     PublisherState(topic='/robot/end_effector/right_gripper/command',
        #             datatype=EndEffectorCommand,
        #             data={'command':'go', 'sender':'user', 'id': 65537, 'args':'{\"grip_attempt_seconds\": 100}'},
        #             # data={'command':'go', 'sender':'user', 'id': 65538, 'args':'{\"grip_attempt_seconds\": 100}'},
        #             n=5, hz=10),
        #     transitions={'succeeded':'move_arm_to_object', 'aborted':'abort_state'})

        # retreat from the bin in a straight line
        # sm.add('remove_object_from_bin', MoveRobotState(),
        #      transitions={'succeeded':'move_arm_out_of_bin'})

        # move to object tf
        sm.add('move_arm_to_object', MoveRobotState(movegroup='left_arm', velocity_scale=0.5),
            # transitions={'succeeded':'move_arm_out_of_bin',
            transitions={'succeeded':'turn_off_the_shelf_collision_scene',
                         'failed':'deactivate_suction',  # Infinite loop, state must say aborted. Don't think this happens yet
                         'aborted':'deactivate_suction'})
        # sm.add('move_arm_to_object', GraspObjectState(velocity_scale=0.5),
        #     # transitions={'succeeded':'move_arm_out_of_bin',
        #     transitions={'succeeded':'turn_off_the_shelf_collision_scene',
        #                  'failed':'abort_state',  # Infinite loop, state must say aborted. Don't think this happens yet
        #                  'aborted':'abort_state'})

        sm.add('turn_off_the_shelf_collision_scene', ToggleBinFillersAndTote(action='unfill_bins'),
            transitions={'succeeded':'lift_object_up',
             'failed': 'abort_state'})

        sm.add('lift_object_up', MoveRobotToRelativePose(movegroup='left_arm',
                            pose_frame_id='/shelf', relative_pose=Pose(position=Point(-0.10,0,0),orientation=Quaternion(0,0,0,1)),
                            velocity_scale=1.0),
             transitions={'succeeded':'move_object_back_out',
                          'failed':'abort_state'})

        sm.add('move_object_back_out', MoveRobotToRelativePose(movegroup='left_arm',
                            pose_frame_id='/shelf', relative_pose=Pose(position=Point(0,0,-0.3),orientation=Quaternion(0,0,0,1)),
                            velocity_scale=0.5),
             transitions={'succeeded':'reset_the_shelf_collision_scene',
                          'failed':'abort_state'})

        sm.add('reset_the_shelf_collision_scene', ToggleBinFillersAndTote(action='fill_bins'),
            transitions={'succeeded':'add_object_collision',
             'failed': 'abort_state'})

        sm.add('add_object_collision', UpdateCollisionState(action='attach'),
            transitions={'succeeded':'move_to_drop_pose'})

        # sm.add('move_sucker_infront_of_bin_2', MoveRobotState(movegroup='left_arm',
        #     named=True, name_prefix='ready_to_suck_in_'),
        #     transitions={'succeeded':'move_to_drop_pose',
        #                  'failed': 'abort_state',
        #                  'aborted':'abort_state'})

        # move the object on top of the tote to drop it
        sm.add('move_to_drop_pose', MoveRobotToNamedPose(movegroup='left_arm',
                                        goal_pose_name='left_tote'),
            transitions={'succeeded':'deactivate_suction',
                         'aborted': 'move_to_neutral_1',
                         'failed': 'move_to_neutral_1'})

        sm.add('move_to_neutral_1', MoveRobotToNamedPose(movegroup='left_arm',
                                goal_pose_name='left_neutral'),
            transitions={'succeeded':'deactivate_suction',
                         'failed': 'temp_remove_object_collision'})

        sm.add('temp_remove_object_collision', UpdateCollisionState(action='detach'),
            transitions={'succeeded':'abort_state'})

        sm.add('deactivate_suction', SuctionState(state='off', movegroup=None),
            transitions={'succeeded':'remove_current_pick', 'failed':'abort_state'})

        sm.add('remove_current_pick', PopItemState(),
            transitions={'succeeded':'move_to_neutral'})

        sm.add('move_to_neutral', MoveRobotToNamedPose(movegroup='left_arm',
                                    goal_pose_name='left_neutral'),
            transitions={'succeeded':'decide_next_pick',
                         'failed': 'abort_state'})




        sm.add('abort_state', PopItemState(),
            transitions={'succeeded':'succeeded'})


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
