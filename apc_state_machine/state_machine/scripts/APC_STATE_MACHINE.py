#! /usr/bin/env python
import rospy
import smach
import smach_ros

import threading

# from findObject2DState import FindObject2DState
from abortState import AbortState
from decideGraspPoseStateFromPointCloudGP import DecideGraspPoseStateFromPointCloud
from decideNextPickItemState import DecideNextPickItemState
from moveRobotToNamedPose import MoveRobotToNamedPose
from moveRobotToRelativePose import MoveRobotToRelativePose
from popItemState import PopItemState
from publisherState import PublisherState
from graspObjectState import GraspObjectState
from toggleBinFillersAndTote import ToggleBinFillersAndTote
from updateCollisionState import UpdateCollisionState
from suctionState import SuctionState
from scanShelfState import ScanShelfState
from getKinfuCloud import GetKinfuCloud
from shelfBasedCropCloud import ShelfBasedCropCloud
from segmentPointcloud import SegmentPointcloud
from objectProposals import ObjectProposals
from preFillUserDataState import PreFillUserDataState

from geometry_msgs.msg import Pose, Point, Quaternion
from baxter_core_msgs.msg import EndEffectorCommand
from checkPressureSensorState import CheckPressureSensorState
from googlenetState import GooglenetState
from pickSuccessfulState import PickSuccessfulState
from pickUnsuccessfulState import PickUnsuccessfulState
from trackItemGraspAttemptsState import TrackItemGraspAttemptsState

# =============================================================================
if __name__ == '__main__':
    rospy.init_node('APC_STATE_MACHINE')

    which_arm = rospy.get_param("/apc_global/which_arm","left_arm")

    if which_arm[:4] == "left":
        limb = "left_"
    else:
        limb = "right_"

    sm_init = smach.StateMachine(outcomes=['repeat','perception','abort_next_object'],
        output_keys=['next_item_to_pick','current_item_attempts'])

    with sm_init:

        sm_init.add('initialise_grasping_userdata', PreFillUserDataState({'current_item_attempts':0}),
            transitions={'succeeded':'deactivate_suction_init',
            'failed':'initialise_grasping_userdata'},
            remapping={'map_key_0':'current_item_attempts'})

        sm_init.add('deactivate_suction_init',
           PublisherState(topic='/robot/end_effector/right_gripper/command',
                   datatype=EndEffectorCommand,
                   data={'command':'stop', 'sender':'user', 'id': 65537},
                   n=5, hz=10),
                   transitions={'succeeded':'decide_next_pick','aborted':'repeat'})

        sm_init.add('decide_next_pick', DecideNextPickItemState(),
           transitions={'succeeded':'set_the_shelf_collision_scene_init',
           'finished':'abort_next_object','aborted':'abort_next_object'})

        sm_init.add('set_the_shelf_collision_scene_init',
            ToggleBinFillersAndTote(action='fill_bins'),
               transitions={'succeeded':'set_the_tote_collision_scene',
                'failed': 'repeat'})

        sm_init.add('set_the_tote_collision_scene',
            ToggleBinFillersAndTote(action='all_on'),
               transitions={'succeeded':'move_to_neutral_start',
               'failed': 'repeat'})

        sm_init.add('move_to_neutral_start', MoveRobotToNamedPose(movegroup=limb + 'arm',
                                   goal_pose_name=limb + 'neutral'),
               transitions={'succeeded':'perception',
                            'failed': 'repeat'})

    sm_perception = smach.StateMachine(outcomes=['repeat','grasping',
        'abort_next_object'],input_keys=['next_item_to_pick'],
        output_keys=['googlenet'])

    with sm_perception:

        sm_perception.add('toggle_lip_off', ToggleBinFillersAndTote(action='lip_off'),
           transitions={'succeeded':'shelf_scan',
            'failed': 'shelf_scan'})

        sm_perception.add('shelf_scan',ScanShelfState(),
            transitions={'succeeded':'toggle_lip_on','failed':'repeat'})

        sm_perception.add('toggle_lip_on', ToggleBinFillersAndTote(action='lip_on'),
           transitions={'succeeded':'get_kinfu_cloud',
            'failed': 'repeat'})

        sm_perception.add('get_kinfu_cloud',GetKinfuCloud(),
           transitions={'succeeded':'crop_kinfu_cloud','failed':'repeat'})

        sm_perception.add('crop_kinfu_cloud',ShelfBasedCropCloud(),
           transitions={'succeeded':'segment_cloud','failed':'repeat'})

        sm_perception.add('segment_cloud',SegmentPointcloud(),
           transitions={'succeeded':'object_proposals','failed':'repeat'})

        sm_perception.add('object_proposals',ObjectProposals(),
           transitions={'succeeded':'googlenet_classify','failed':'repeat'})

        sm_perception.add('googlenet_classify',GooglenetState(),
           transitions={'succeeded':'grasping','failed':'repeat'})

    sm_grasp = smach.StateMachine(outcomes=['repeat','perception','move_tote',
    'abort_next_object'],
        input_keys=['next_item_to_pick','googlenet','current_item_attempts'],
        output_keys=['move_group','current_item_attempts'])

    with sm_grasp:

        sm_grasp.add('decide_grasp_pose', DecideGraspPoseStateFromPointCloud(),
            transitions={'succeeded':'toggle_lip_off',
                         'failed':'track_number_of_grasp_attempts'})

        sm_grasp.add('toggle_lip_off', ToggleBinFillersAndTote(action='lip_off'),
           transitions={'succeeded':'move_sucker_infront_of_bin',
            'failed': 'track_number_of_grasp_attempts'})

        sm_grasp.add('move_sucker_infront_of_bin', MoveRobotToNamedPose(movegroup=limb + 'arm',
                        name_prefix=limb + 'ready_to_suck_in_'),
            transitions={'succeeded':'toggle_lip_on',
                         'failed': 'track_number_of_grasp_attempts'})

        sm_grasp.add('toggle_lip_on', ToggleBinFillersAndTote(action='lip_on'),
           transitions={'succeeded':'move_arm_to_object',
            'failed': 'track_number_of_grasp_attempts'})

        sm_grasp.add('move_arm_to_object', GraspObjectState(velocity_scale=0.5),
            transitions={'succeeded':'turn_off_the_shelf_collision_scene',
                         'failed':'turn_off_the_shelf_collision_scene_failed',
                         'aborted':'turn_off_the_shelf_collision_scene_failed'})

        sm_grasp.add('turn_off_the_shelf_collision_scene',
            ToggleBinFillersAndTote(action='unfill_bins'),
             transitions={'succeeded':'lift_object_up',
              'failed': 'turn_off_the_shelf_collision_scene'})

        sm_grasp.add('turn_off_the_shelf_collision_scene_failed',
          ToggleBinFillersAndTote(action='unfill_bins'),
           transitions={'succeeded':'move_object_back_out',
            'failed': 'turn_off_the_shelf_collision_scene_failed'})


        sm_grasp.add('lift_object_up',
            MoveRobotToRelativePose(movegroup=limb + 'arm_cartesian',
                                    pose_frame_id='/shelf',
                                    relative_pose=
                                        Pose(position=Point(-0.10,0,0),
                                            orientation=Quaternion(0,0,0,1)),
                                    velocity_scale=1.0),
                                    transitions={'succeeded':'check_pressure_sensor',
                                    'failed':'move_object_back_out','aborted':'move_object_back_out'})

        sm_grasp.add('move_object_back_out', MoveRobotToRelativePose(movegroup=limb + 'arm_cartesian',
                         pose_frame_id='/shelf', relative_pose=Pose(position=Point(0,0,-0.45),orientation=Quaternion(0,0,0,1)),
                         velocity_scale=1.0),
          transitions={'succeeded':'reset_the_shelf_collision_scene',
                       'failed':'track_number_of_grasp_attempts','aborted':'track_number_of_grasp_attempts'})

        sm_grasp.add('move_back_out_no_object', MoveRobotToRelativePose(movegroup=limb + 'arm_cartesian',
                         pose_frame_id='/shelf', relative_pose=Pose(position=Point(0,0,-0.45),orientation=Quaternion(0,0,0,1)),
                         velocity_scale=1.0),
          transitions={'succeeded':'reset_the_shelf_collision_scene_failed',
                       'failed':'track_number_of_grasp_attempts','aborted':'track_number_of_grasp_attempts'})

        sm_grasp.add('reset_the_shelf_collision_scene', ToggleBinFillersAndTote(action='fill_bins'),
             transitions={'succeeded':'turn_lip_off',
              'failed': 'reset_the_shelf_collision_scene'})

        sm_grasp.add('reset_the_shelf_collision_scene_failed', ToggleBinFillersAndTote(action='fill_bins'),
             transitions={'succeeded':'turn_lip_off_failed',
              'failed': 'reset_the_shelf_collision_scene_failed'})

        sm_grasp.add('turn_lip_off', ToggleBinFillersAndTote(action='lip_off'),
            transitions={'succeeded':'move_tote',
            'failed': 'turn_lip_off'})

        sm_grasp.add('turn_lip_off_failed', ToggleBinFillersAndTote(action='lip_off'),
            transitions={'succeeded':'track_number_of_grasp_attempts',
            'failed': 'turn_lip_off_failed'})

        sm_grasp.add('check_pressure_sensor', CheckPressureSensorState(),transitions={'succeeded':'move_object_back_out',
                     'failed':'pressure_failed_suction_off'})

        sm_grasp.add('pressure_failed_suction_off', SuctionState(state='off', movegroup=None),
            transitions={'succeeded':'move_back_out_no_object', 'failed':'track_number_of_grasp_attempts'})

        sm_grasp.add('track_number_of_grasp_attempts', TrackItemGraspAttemptsState(),
            transitions={'try_this_item_again':'perception', 'move_on_to_next_item':'abort_next_object'})

    sm_move_tote = smach.StateMachine(outcomes=['repeat','perception','finished_next_object'],input_keys=['next_item_to_pick','move_group'])

    with sm_move_tote:

        sm_move_tote.add('move_to_drop_pose', MoveRobotToNamedPose(movegroup=limb + 'arm',
                                        goal_pose_name=limb + 'tote'),
            transitions={'succeeded':'turn_tote_off',
                         'failed': 'move_to_neutral_drop_failed'})

        sm_move_tote.add('turn_tote_off', ToggleBinFillersAndTote(action='tote_off'),
           transitions={'succeeded':'lower_into_tote',
            'failed': 'turn_tote_off'})

        sm_move_tote.add('lower_into_tote', MoveRobotToNamedPose(movegroup=limb + 'arm',
                                                         goal_pose_name=limb + 'tote_drop'),
                             transitions={'succeeded':'check_pressure_sensor',
                                          'failed': 'move_to_neutral_drop_failed'})

        sm_move_tote.add('move_to_neutral_drop_failed', MoveRobotToNamedPose(movegroup=limb + 'arm',
                                             goal_pose_name=limb + 'neutral'),
                     transitions={'succeeded':'move_to_drop_pose',
                                  'failed': 'move_to_neutral_drop_failed'})

        sm_move_tote.add('check_pressure_sensor', CheckPressureSensorState(),transitions={'succeeded':'remove_current_pick',
                     'failed':'pick_unsuccessful'}) # TODO could add in update a list of lost items perhaps

        sm_move_tote.add('pick_unsuccessful', PickUnsuccessfulState(),
        transitions={'succeeded':'deactivate_suction','aborted':'deactivate_suction'})

        sm_move_tote.add('remove_current_pick', PickSuccessfulState(),
        transitions={'succeeded':'deactivate_suction','aborted':'remove_current_pick'})

        sm_move_tote.add('deactivate_suction', SuctionState(state='off', movegroup=None),
            transitions={'succeeded':'remove_object_collision', 'failed':'deactivate_suction'})

        sm_move_tote.add('remove_object_collision', UpdateCollisionState(action='detach'),
            transitions={'succeeded':'move_back_to_drop_pose','aborted':'remove_object_collision'})

        sm_move_tote.add('move_back_to_drop_pose', MoveRobotToNamedPose(movegroup=limb + 'arm',
                                            goal_pose_name=limb + 'tote'),
                transitions={'succeeded':'turn_tote_on', 'failed': 'move_back_to_drop_pose'})

        sm_move_tote.add('turn_tote_on', ToggleBinFillersAndTote(action='tote_on'),
           transitions={'succeeded':'move_to_neutral',
            'failed': 'turn_tote_on'})

        sm_move_tote.add('move_to_neutral', MoveRobotToNamedPose(movegroup=limb + 'arm',
                                    goal_pose_name=limb + 'neutral'),
            transitions={'succeeded':'finished_next_object',
                         'failed': 'move_to_neutral'})

    # # Create the top level SMACH state machine
    # sm_move_bin = smach.StateMachine(outcomes=['repeat','perception','move_to_tote','abort_next_object'])
    #
    # # create states and connect them
    # with sm_move_bin:
    #
    #     # move the object on top of the tote to drop it
    #     sm_move_bin.add('move_to_drop_pose', MoveRobotToNamedPose(movegroup=limb + 'arm',
    #                                     goal_pose_name=limb + 'tote'),
    #         transitions={'succeeded':'deactivate_suction',
    #                      'aborted': 'move_to_neutral_1',
    #                      'failed': 'move_to_neutral_1'})
    #
    #     sm_move_bin.add('move_to_neutral_1', MoveRobotToNamedPose(movegroup=limb + 'arm',
    #                             goal_pose_name=limb + 'neutral'),
    #         transitions={'succeeded':'deactivate_suction',
    #                      'failed': 'temp_remove_object_collision'})
    #
    #     sm_move_bin.add('temp_remove_object_collision', UpdateCollisionState(action='detach'),
    #         transitions={'succeeded':'abort_state'})
    #
    #     sm_move_bin.add('deactivate_suction', SuctionState(state='off', movegroup=None),
    #         transitions={'succeeded':'remove_object_collision', 'failed':'abort_state'})
    #
    #     sm_move_bin.add('remove_object_collision', UpdateCollisionState(action='detach'),
    #         transitions={'succeeded':'remove_current_pick'})
    #
    #     sm_move_bin.add('remove_current_pick', PopItemState(),
    #         transitions={'succeeded':'move_to_neutral'})
    #
    #     sm_move_bin.add('move_to_neutral', MoveRobotToNamedPose(movegroup=limb + 'arm',
    #                                 goal_pose_name=limb + 'neutral'),
    #         transitions={'succeeded':'decide_next_pick',
    #                      'failed': 'abort_state'})


    # TOP LEVEL STATE MACHINE
    sm = smach.StateMachine(outcomes=['succeeded', 'aborted'])

    with sm:
        sm.add('INITIAL', sm_init, transitions={'repeat':'INITIAL','perception':'PERCEPTION','abort_next_object':'POP'},remapping={'next_item_to_pick':'next_item_to_pick', 'current_item_attempts':'current_item_attempts'})#,'abort_all':'ABORT'})
        sm.add('PERCEPTION', sm_perception, transitions={'repeat':'PERCEPTION','grasping':'GRASPING','abort_next_object':'POP'},remapping={'googlenet':'googlenet', 'current_item_attempts':'current_item_attempts'})#,'abort_all':'ABORT'})
        sm.add('GRASPING', sm_grasp, transitions={'repeat':'GRASPING','perception':'PERCEPTION','move_tote':'MOVE_TOTE','abort_next_object':'POP'},remapping={'move_group':'move_group', 'current_item_attempts':'current_item_attempts'})#,'abort_all':'ABORT'})
        sm.add('MOVE_TOTE', sm_move_tote, transitions={'repeat':'MOVE_TOTE','perception':'PERCEPTION','finished_next_object':'INITIAL'})#,'abort_all':'ABORT'})
        # sm.add('MOVE_BIN', sm_move_bin, transitions={})

        sm.add('ABORT', AbortState(),
              transitions={'succeeded':'succeeded'})
        sm.add('POP', PopItemState(),
            transitions={'succeeded':'INITIAL'})


    # Create and start the introspection server
    #  (smach_viewer is broken in indigo + 14.04, so need for that now)
    sis = smach_ros.IntrospectionServer('server_name', sm, '/APC_SM')
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
