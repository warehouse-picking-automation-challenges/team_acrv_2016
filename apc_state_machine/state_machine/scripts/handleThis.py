#! /usr/bin/env python

import roslib
import rospy
import smach
import smach_ros

import threading

from abortState import AbortState
from decideGraspPoseStateFromPointCloud import DecideGraspPoseStateFromPointCloud
from decideNextPickItemState import DecideNextPickItemState
# from detectObjectState import DetectObjectState
from moveRobotState import MoveRobotState
from moveRobotToNamedPose import MoveRobotToNamedPose
from moveRobotToRelativePose import MoveRobotToRelativePose
from popItemState import PopItemState
from publisherState import PublisherState
from getBinCoordinatesState import GetBinCoordinatesState
from graspObjectState import GraspObjectState
from toggleBinFillersAndTote import ToggleBinFillersAndTote
from updateCollisionState import UpdateCollisionState
from chooseTheNextItemToPick import ChooseTheNextItemToPick
from chooseWhereToGoState import ChooseWhereToGoState
from createMeAPickPlanState import CreateMeAPickPlanState
from detectAllObjectsState import DetectAllObjectsState
from removeObjectFromPickPlan import RemoveObjectFromPickPlan
# import decideGraspPoseStateFromPointCloud
from suctionState import SuctionState
import waitState
from scanShelfState import ScanShelfState
from getKinfuCloud import GetKinfuCloud
from shelfBasedCropCloud import ShelfBasedCropCloud

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
        # deactivate the suction, just to make sure that no errors since last run
        sm.add('deactivate_suction_init',
            PublisherState(topic='/robot/end_effector/right_gripper/command',
                    datatype=EndEffectorCommand,
                    data={'command':'stop', 'sender':'user', 'id': 65537},
                    n=5, hz=10),
            transitions={'succeeded':'decide_next_pick'})

        sm.add('decide_next_pick', DecideNextPickItemState(),
            transitions={'succeeded':'reset_the_shelf_collision_scene_init', 'finished': 'succeeded'})

        sm.add('reset_the_shelf_collision_scene_init', ToggleBinFillersAndTote(action='fill_bins'),
            transitions={'succeeded':'move_to_neutral_start',
             'failed': 'abort_state'})

        sm.add('move_to_neutral_start', MoveRobotToNamedPose(movegroup='left_arm',
                                    goal_pose_name='left_neutral'),
                transitions={'succeeded':'move_arm_to_look_into_bin',    # 'lift_object_up'
                             'failed': 'abort_state'})

        # sm.add('remove_object_collision', UpdateCollisionState(action='detach'),
        #     transitions={'succeeded':'set_the_shelf_collision_scene'})

       # fill unused bins with collision objects
       # actions: fill_bins, unfill_bins
        # sm.add('set_the_shelf_collision_scene', ToggleBinFillersAndTote(action='fill_bins'),
        #     transitions={'succeeded':'set_the_tote_collision_scene',
        #      'failed': 'abort_state'})
        #
        # # add tote and tote pillar to collision scene
        # # tote/pillar actions: tote_on, tote_off, pillar_on, pillar_off, all_on, all_off
        # sm.add('set_the_tote_collision_scene', ToggleBinFillersAndTote(action='all_on'),
        #     transitions={'succeeded':'get_target_bin_coordinates',
        #      'failed': 'abort_state'})

        # move arm in front of bin before looking for the object
        # sm.add('get_target_bin_coordinates', GetBinCoordinatesState(),
        #     transitions={'succeeded':'move_arm_to_look_into_bin','aborted':'abort_state'})

        # sm.add('move_arm_to_look_into_bin', MoveRobotState(movegroup='left_arm_realsense'),
        sm.add('move_arm_to_look_into_bin', MoveRobotToNamedPose(movegroup='left_arm',
                                    name_prefix='look_into_'),
            transitions={'succeeded':'detect_all_objects',
                         'failed': 'abort_state',
                         'aborted':'abort_state'})
            # transitions={'succeeded':'remove_current_pick', 'aborted':'abort_state'})

        sm.add('detect_all_objects',DetectAllObjectsState(),
            transitions={'succeeded':'create_me_a_plan',
                            'no_objects':'decide_next_pick',
                            'confidence_too_low':'detect_all_objects'})

        sm.add('create_me_a_plan',CreateMeAPickPlanState(),
            transitions={'succeeded':'choose_the_next_item_to_pick'})

        sm.add('choose_the_next_item_to_pick',ChooseTheNextItemToPick(),
            transitions={'succeeded':'toggle_lip_off'})


        sm.add('toggle_lip_off', ToggleBinFillersAndTote(action='lip_off'),
            transitions={'succeeded':'shelf_scan',
             'failed': 'abort_state'})

        sm.add('shelf_scan',ScanShelfState(),
            transitions={'succeeded':'toggle_lip_on','failed':'abort_state'})

        sm.add('toggle_lip_on', ToggleBinFillersAndTote(action='lip_on'),
            transitions={'succeeded':'get_kinfu_cloud',
             'failed': 'abort_state'})

        sm.add('get_kinfu_cloud',GetKinfuCloud(),
            transitions={'succeeded':'crop_kinfu_cloud','failed':'abort_state'})

        sm.add('crop_kinfu_cloud',ShelfBasedCropCloud(),
            transitions={'succeeded':'decide_grasp_pose','failed':'abort_state'})

        sm.add('decide_grasp_pose', DecideGraspPoseStateFromPointCloud(),
            transitions={'succeeded':'move_sucker_infront_of_bin',
                         'failed':'detect_all_objects'})

        # move the sucker into the object, colliding with it slightly\
        # still problems with the planner in here
        sm.add('move_sucker_infront_of_bin', MoveRobotToNamedPose(movegroup='left_arm',
                        name_prefix='ready_to_suck_in_'),
            transitions={'succeeded':'move_arm_to_object',
                         'failed': 'abort_state',
                         'aborted':'abort_state'})

        sm.add('move_arm_to_object', GraspObjectState(velocity_scale=0.5),
            # transitions={'succeeded':'move_arm_out_of_bin',
            transitions={'succeeded':'turn_off_the_shelf_collision_scene',
                         'failed':'move_arm_to_look_into_bin',
                         'aborted':'abort_state'})

        sm.add('turn_off_the_shelf_collision_scene', ToggleBinFillersAndTote(action='unfill_bins'),
            transitions={'succeeded':'lift_object_up',
             'failed': 'abort_state'})

        sm.add('lift_object_up', MoveRobotToRelativePose(movegroup='left_arm_cartesian',
                                                         pose_frame_id='/shelf',
                                                         relative_pose=
                                        Pose(position=Point(-0.10,0,0),
                                             orientation=Quaternion(0,0,0,1)),
                                                        velocity_scale=1.0),
            transitions={'succeeded':'move_object_back_out',
                         'failed':'abort_state'})

        sm.add('move_object_back_out', MoveRobotToRelativePose(movegroup='left_arm_cartesian',
                            pose_frame_id='/shelf', relative_pose=Pose(position=Point(0,0,-0.3),orientation=Quaternion(0,0,0,1)),
                            velocity_scale=1.0),
             transitions={'succeeded':'reset_the_shelf_collision_scene',
                          'failed':'reset_the_shelf_collision_scene'})

        sm.add('reset_the_shelf_collision_scene', ToggleBinFillersAndTote(action='fill_bins'),
            transitions={'succeeded':'add_object_collision',
             'failed': 'abort_state'})

        sm.add('add_object_collision', UpdateCollisionState(action='attach'),
            transitions={'succeeded':'choose_where_to_go'})

        sm.add('choose_where_to_go',ChooseWhereToGoState(),
            transitions={'bin':'move_sucker_infront_of_bin_bincase',
                            'tote':'move_to_drop_pose'})

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
            transitions={'succeeded':'remove_object_collision', 'failed':'abort_state'})

        sm.add('remove_object_collision', UpdateCollisionState(action='detach'),
            transitions={'succeeded':'remove_current_pick'})

        sm.add('remove_current_pick', PopItemState(),
            transitions={'succeeded':'move_to_neutral'})

        sm.add('move_to_neutral', MoveRobotToNamedPose(movegroup='left_arm',
                                    goal_pose_name='left_neutral'),
            transitions={'succeeded':'decide_next_pick',
                         'failed': 'abort_state'})



        #For the bin state
        sm.add('move_sucker_infront_of_bin_bincase', MoveRobotToNamedPose(movegroup='left_arm',
                        name_prefix='ready_to_suck_in_'),
            transitions={'succeeded':'remove_object_collision_bincase',
                         'failed': 'abort_state',
                         'aborted':'abort_state'})

        sm.add('remove_object_collision_bincase', UpdateCollisionState(action='detach'),
            transitions={'succeeded':'set_the_shelf_collision_scene_bincase_dump_pos'})

        sm.add('set_the_shelf_collision_scene_bincase_dump_pos', ToggleBinFillersAndTote(action='fill_bins'),
            transitions={'succeeded':'move_into_bincase',
             'failed': 'abort_state'})

        sm.add('move_into_bincase', MoveRobotToRelativePose(movegroup='left_arm_cartesian',
                                                         pose_frame_id='/shelf',
                                                         relative_pose=
                                        Pose(position=Point(-0.15,0,0.4), #variable here ADAM
                                             orientation=Quaternion(0,0,0,1)),
                                                        velocity_scale=1.0),
            transitions={'succeeded':'deactivate_suction_bincase',
                         'failed':'deactivate_suction_bincase'})

        sm.add('deactivate_suction_bincase', SuctionState(state='off', movegroup=None),
            transitions={'succeeded':'move_out_of_bin_bincase', 'failed':'abort_state'})

        sm.add('move_out_of_bin_bincase', MoveRobotToRelativePose(movegroup='left_arm_cartesian',
                            pose_frame_id='/shelf', relative_pose=Pose(position=Point(0,0,-0.3),orientation=Quaternion(0,0,0,1)),
                            velocity_scale=1.0),
             transitions={'succeeded':'remove_object_from_pick_plan_bincase',
                          'failed':'remove_object_from_pick_plan_bincase'})

        sm.add('remove_object_from_pick_plan_bincase', RemoveObjectFromPickPlan(),
            transitions={'succeeded':'set_the_shelf_collision_scene_bincase'})

        sm.add('set_the_shelf_collision_scene_bincase', ToggleBinFillersAndTote(action='fill_bins'),
            transitions={'succeeded':'set_the_tote_collision_scene_bincase',
             'failed': 'abort_state'})

        # add tote and tote pillar to collision scene
        # tote/pillar actions: tote_on, tote_off, pillar_on, pillar_off, all_on, all_off
        sm.add('set_the_tote_collision_scene_bincase', ToggleBinFillersAndTote(action='all_on'),
            transitions={'succeeded':'get_target_bin_coordinates_bincase',
             'failed': 'abort_state'})

        # move arm in front of bin before looking for the object
        sm.add('get_target_bin_coordinates_bincase', GetBinCoordinatesState(),
            transitions={'succeeded':'move_arm_to_look_into_bin_bincase','aborted':'abort_state'})

        # sm.add('move_arm_to_look_into_bin', MoveRobotState(movegroup='left_arm_realsense'),
        sm.add('move_arm_to_look_into_bin_bincase', MoveRobotToNamedPose(movegroup='left_arm',
                                    name_prefix='look_into_'),
            transitions={'succeeded':'choose_the_next_item_to_pick',
                         'failed': 'abort_state',
                         'aborted':'abort_state'})


        sm.add('abort_state', AbortState(),
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
