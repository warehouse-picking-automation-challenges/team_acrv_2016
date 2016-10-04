#! /usr/bin/env python
import rospy
import smach
import smach_ros

import threading

# from findObject2DState import FindObject2DState
from abortState import AbortState
from decideGraspPoseStateFromPointCloudGP import DecideGraspPoseStateFromPointCloud
from moveRobotToNamedPose import MoveRobotToNamedPose
from moveRobotToRelativePose import MoveRobotToRelativePose
from publisherState import PublisherState
from graspObjectState import GraspObjectState
from toggleBinFillersAndTote import ToggleBinFillersAndTote
from suctionState import SuctionState
from getKinfuCloud import GetKinfuCloud
from getKinectCloud import GetKinectCloud
from shelfBasedCropCloud import ShelfBasedCropCloud
from segmentPointcloud import SegmentPointcloud
from objectProposals import ObjectProposals
from toggleRealsenseKinectJake import ToggleRealsenseKinectJake
from waitState import WaitState
from getToteContents import GetToteContents
from updateCollisionState import UpdateCollisionState

from geometry_msgs.msg import Pose, Point, Quaternion
from baxter_core_msgs.msg import EndEffectorCommand
from checkPressureSensorState import CheckPressureSensorState
from googlenetState import GooglenetState
from stowSuccessfulState import StowSuccessfulState
from addObjectWeightState import AddObjectWeight
from filterRedCloud import FilterRedCloudUpdateToteTF
from decideBinForStow import DecideBinForStow

# =============================================================================
if __name__ == '__main__':
    rospy.init_node('getToteData')

    sm_init = smach.StateMachine(outcomes=['repeat','perception','abort_next_object'],
        output_keys=['next_item_to_pick'])

    with sm_init:

        sm_init.add('deactivate_suction_init',
            PublisherState(topic='/robot/end_effector/right_gripper/command',
            datatype=EndEffectorCommand,
            data={'command':'stop', 'sender':'user', 'id': 65537},
            n=5, hz=10),
            transitions={'succeeded':'get_tote_objects','aborted':'repeat'})

        sm_init.add('get_tote_objects',GetToteContents(), transitions={'succeeded':'turn_on_tote_and_podium','aborted':'repeat'})

        sm_init.add('turn_on_tote_and_podium',
            ToggleBinFillersAndTote(action='all_on'),
            transitions={'succeeded':'turn_on_tote_and_podium_2',
             'failed': 'turn_on_tote_and_podium'})
        #MUAHAHAHAHA
        sm_init.add('turn_on_tote_and_podium_2',
            ToggleBinFillersAndTote(action='all_on'),
            transitions={'succeeded':'set_the_shelf_collision_scene',
             'failed': 'turn_on_tote_and_podium_2'})

        sm_init.add('set_the_shelf_collision_scene', ToggleBinFillersAndTote(action='fill_bins_tote'),
             transitions={'succeeded':'move_to_look_into_tote_pose',
              'failed': 'set_the_shelf_collision_scene'})

        sm_init.add('move_to_left_tote', MoveRobotToNamedPose(movegroup='left_arm',
                                        goal_pose_name='left_tote'),
            transitions={'succeeded':'move_to_look_into_tote_pose',
                         'aborted': 'repeat',
                         'failed': 'move_to_left_tote'})

        sm_init.add('move_to_look_into_tote_pose', MoveRobotToNamedPose(movegroup='left_arm',
                                               goal_pose_name='look_into_tote'),
                   transitions={'succeeded':'perception',
                                'aborted': 'repeat',
                                'failed': 'move_to_look_into_tote_pose'})


    sm_perception = smach.StateMachine(outcomes=['repeat','grasping',
        'abort_next_object'],input_keys=['next_item_to_pick'],output_keys=['googlenet','next_item_to_pick'])

    with sm_perception:

        ## LOCALISE TOTE FROM REALSENSE LIVE? OR FROM REALSENSE KINFU?

        # MAY NEED TO MODIFY FOR TOTE
        sm_perception.add('reset_kinfu',
            ToggleRealsenseKinectJake(action='realsense_on_kinect_off'),
            transitions={'succeeded':'wait_sync1','failed':'reset_kinfu'})

        sm_perception.add('wait_sync1', WaitState(2.0),
            transitions={'succeeded':'get_kinfu_cloud','preempted':'get_kinfu_cloud'})

        sm_perception.add('get_kinfu_cloud',GetKinfuCloud(),
           transitions={'succeeded':'remove_red_cloud_update_tote_tf','failed':'abort_next_object'})

        sm_perception.add('remove_red_cloud_update_tote_tf',FilterRedCloudUpdateToteTF(),
            transitions={'succeeded':'crop_kinfu_cloud','failed':'crop_kinfu_cloud'})

        # NEED TO ENSURE TOTE TF EXISTS and MODIFY NODE TO HAVE DIFFERENT CROPPING CONDITIONS FOR TOTE
        # ALTERNATELY reject cloud points based on colour
        sm_perception.add('crop_kinfu_cloud',ShelfBasedCropCloud(action='crop_tote'),
           transitions={'succeeded':'segment_cloud','failed':'repeat'})

        sm_perception.add('segment_cloud',SegmentPointcloud(),
           transitions={'succeeded':'object_proposals','failed':'repeat'})

        sm_perception.add('object_proposals',ObjectProposals(),
           transitions={'succeeded':'googlenet','failed':'repeat'})

        sm_perception.add('googlenet',GooglenetState(action='at_tote'),
           transitions={'succeeded':'repeat','failed':'repeat'})

    sm_grasp = smach.StateMachine(outcomes=['repeat','init','perception','googlenet'
        ,'abort_next_object'],
        input_keys=['next_item_to_pick','googlenet'],
        output_keys=['move_group'])

    with sm_grasp:

        sm_grasp.add('turn_off_the_tote_1',
            ToggleBinFillersAndTote(action='all_off'),
            transitions={'succeeded':'decide_grasp_pose',
            'failed': 'turn_off_the_tote_1'})

        ## FORCE front gripper grasps?
        sm_grasp.add('decide_grasp_pose', DecideGraspPoseStateFromPointCloud(),
            transitions={'succeeded':'turn_on_the_tote',
                         'failed':'abort_next_object'})

        sm_grasp.add('turn_on_the_tote',
             ToggleBinFillersAndTote(action='all_on'),
             transitions={'succeeded':'move_to_drop_pose',
             'failed': 'turn_on_the_tote'})

        sm_grasp.add('move_to_drop_pose', MoveRobotToNamedPose(movegroup='left_arm',
                                     goal_pose_name='left_tote'),
                                    transitions={'succeeded':'turn_off_the_tote',
                                      'aborted': 'abort_next_object',
                                      'failed': 'repeat'})

        sm_grasp.add('turn_off_the_tote',
                     ToggleBinFillersAndTote(action='all_off'),
                      transitions={'succeeded':'move_arm_to_object',
                       'failed': 'turn_off_the_tote'})

        sm_grasp.add('move_arm_to_object', GraspObjectState(velocity_scale=0.5),
            transitions={'succeeded':'lift_object_up',
                         'failed':'init',
                         'aborted':'abort_next_object'})


        sm_grasp.add('lift_object_up', MoveRobotToNamedPose(movegroup='left_arm',
                          goal_pose_name='left_tote'),
                         transitions={'succeeded':'turn_on_the_tote_again',
                           'aborted': 'abort_next_object',
                           'failed': 'repeat'})

        sm_grasp.add('turn_on_the_tote_again',
          ToggleBinFillersAndTote(action='tote_on'),
           transitions={'succeeded':'check_pressure_sensor',
            'failed': 'turn_on_the_tote_again'})

        sm_grasp.add('check_pressure_sensor', CheckPressureSensorState(),transitions={'succeeded':'add_object_collision',
                     'failed':'pressure_failed_suction_off'})

        sm_grasp.add('pressure_failed_suction_off', SuctionState(state='off', movegroup=None),
            transitions={'succeeded':'init', 'failed':'pressure_failed_suction_off'})

        sm_grasp.add('add_object_collision', UpdateCollisionState(action='attach'),
            transitions={'succeeded':'add_object_weight','aborted':'add_object_collision'})

        sm_grasp.add('add_object_weight',AddObjectWeight(),transitions={'succeeded':'move_to_kinect','failed':'add_object_weight'})


        sm_grasp.add('move_to_kinect', MoveRobotToNamedPose(movegroup='left_arm',
                            goal_pose_name='kinect_look',velocity_scale=0.5),
                           transitions={'succeeded':'googlenet',
                             'aborted': 'pressure_failed_suction_off', # SHOULD DROP ITEM BACK IN TOTE?
                             'failed': 'move_to_kinect'})

    sm_googlenet = smach.StateMachine(outcomes=['repeat','perception','move_object_into_bin','abort_next_object'],input_keys=['next_item_to_pick','move_group'])

    with sm_googlenet:

        # SWITCH REALSENSE OFF
        sm_googlenet.add('switch_realsense_off_kinect_on',
            ToggleRealsenseKinectJake(action='realsense_off_kinect_on'),
            transitions={'succeeded':'wait_sync1','failed':'switch_realsense_off_kinect_on'})

        sm_googlenet.add('wait_sync1', WaitState(5.0),
            transitions={'succeeded':'get_kinect_cloud','preempted':'get_kinect_cloud'})

        # GET KINFU CLOUD
        sm_googlenet.add('get_kinect_cloud',GetKinectCloud(),
           transitions={'succeeded':'crop_kinfu_cloud','failed':'abort_next_object'})

        # CROP CLOUD BASED ON Z-DIRECTION, will have some arm remainign
        sm_googlenet.add('crop_kinfu_cloud',ShelfBasedCropCloud(action='crop_kinfu_cloud'),
           transitions={'succeeded':'segment_cloud','failed':'repeat'})

        # SEGMENTATION, WILL NEED TO CHANGE SEGMENTATION PARAMETERS
        sm_googlenet.add('segment_cloud',SegmentPointcloud(),
            transitions={'succeeded':'object_proposals','failed':'repeat'})

        # OBJECT PROPOSALS
        sm_googlenet.add('object_proposals',ObjectProposals(),
            transitions={'succeeded':'googlenet_classify','failed':'repeat'})

        # GOOGLENET
        sm_googlenet.add('googlenet_classify',GooglenetState(action='at_kinect'),
            transitions={'succeeded':'move_object_into_bin','failed':'repeat'})

        # GET BIN REQUIRED TO BE PLACED


    sm_move_object_into_bin = smach.StateMachine(outcomes=['repeat','perception','init','abort_next_object'],input_keys=['next_item_to_pick','move_group'])

    with sm_move_object_into_bin:

        sm_move_object_into_bin.add('decide_bin',DecideBinForStow(),
            transitions={'succeeded':'reset_the_shelf_collision_scene','aborted':'repeat'})

        sm_move_object_into_bin.add('reset_the_shelf_collision_scene', ToggleBinFillersAndTote(action='fill_bins'),
            transitions={'succeeded':'move_sucker_infront_of_bin',
            'failed': 'reset_the_shelf_collision_scene'})

        sm_move_object_into_bin.add('move_sucker_infront_of_bin', MoveRobotToNamedPose(movegroup='left_arm',
           name_prefix='ready_to_suck_in_',velocity_scale=0.5),
           transitions={'succeeded':'move_sucker_up',
            'failed': 'move_sucker_infront_of_bin',
            'aborted':'abort_next_object'})

        sm_move_object_into_bin.add('move_sucker_up', MoveRobotToRelativePose(movegroup='left_arm_cartesian',
                             pose_frame_id='/shelf', relative_pose=Pose(position=Point(-0.05,0,0.0),orientation=Quaternion(0,0,0,1)),
                             velocity_scale=1.0),
              transitions={'succeeded':'move_sucker_into_bin',
                           'failed':'move_sucker_infront_of_bin','aborted':'abort_next_object'})

        sm_move_object_into_bin.add('move_sucker_into_bin', MoveRobotToRelativePose(movegroup='left_arm_cartesian',
                         pose_frame_id='/shelf', relative_pose=Pose(position=Point(0.0,0,0.3),orientation=Quaternion(0,0,0,1)),
                         velocity_scale=1.0),
          transitions={'succeeded':'deactivate_suction_init',
                       'failed':'move_sucker_infront_of_bin','aborted':'abort_next_object'})

        sm_move_object_into_bin.add('deactivate_suction_init',
            PublisherState(topic='/robot/end_effector/right_gripper/command',
            datatype=EndEffectorCommand,
            data={'command':'stop', 'sender':'user', 'id': 65537},
            n=5, hz=10),
            transitions={'succeeded':'remove_object_collision','aborted':'repeat'})

        sm_move_object_into_bin.add('remove_object_collision', UpdateCollisionState(action='detach'),
            transitions={'succeeded':'remove_object_weight','aborted':'remove_object_collision'})

        sm_move_object_into_bin.add('remove_object_weight',AddObjectWeight(action='remove_object_weight'),transitions={'succeeded':'remove_current_stow','failed':'remove_object_weight'})

        sm_move_object_into_bin.add('remove_current_stow', StowSuccessfulState(), transitions={'succeeded':'move_sucker_out_of_bin','aborted':'remove_current_stow'})

        sm_move_object_into_bin.add('set_the_shelf_collision_scene', ToggleBinFillersAndTote(action='fill_bins'),
            transitions={'succeeded':'move_sucker_out_of_bin',
             'failed': 'set_the_shelf_collision_scene'})

        sm_move_object_into_bin.add('move_sucker_out_of_bin', MoveRobotToRelativePose(movegroup='left_arm_cartesian',
                         pose_frame_id='/shelf', relative_pose=Pose(position=Point(0,0,-0.3),orientation=Quaternion(0,0,0,1)),
                         velocity_scale=1.0),
          transitions={'succeeded':'init',
                       'failed':'abort_next_object','aborted':'abort_next_object'})


    # TOP LEVEL STATE MACHINE
    sm = smach.StateMachine(outcomes=['succeeded', 'aborted'])

    with sm:
        sm.add('INITIAL', sm_init, transitions={'repeat':'INITIAL','perception':'PERCEPTION','abort_next_object':'ABORT'},remapping={'next_item_to_pick':'next_item_to_pick'})
        sm.add('PERCEPTION', sm_perception, transitions={'repeat':'PERCEPTION','grasping':'GRASPING','abort_next_object':'INITIAL'},remapping={'googlenet':'googlenet'})
        sm.add('GRASPING', sm_grasp, transitions={'repeat':'GRASPING','init':'INITIAL','perception':'PERCEPTION','googlenet':'GOOGLENET','abort_next_object':'INITIAL'},remapping={'move_group':'move_group'})
        sm.add('GOOGLENET', sm_googlenet, transitions={'repeat':'GOOGLENET','perception':'PERCEPTION','move_object_into_bin':'MOVE_OBJECTINTOBIN','abort_next_object':'ABORT'})
        sm.add('MOVE_OBJECTINTOBIN', sm_move_object_into_bin, transitions={'repeat':'MOVE_OBJECTINTOBIN','init':'INITIAL','perception':'PERCEPTION','abort_next_object':'ABORT'})

        sm.add('ABORT', AbortState(),
              transitions={'succeeded':'succeeded'})


    # Create and start the introspection server
    #  (smach_viewer is broken in indigo + 14.04, so need for that now)
    sis = smach_ros.IntrospectionServer('server_name', sm, '/APC_SM_STOW')
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
