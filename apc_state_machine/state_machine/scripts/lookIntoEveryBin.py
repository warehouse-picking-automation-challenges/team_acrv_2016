#! /usr/bin/env python
import roslib
import rospy
import smach
import smach_ros




import threading

# from findObject2DState import FindObject2DState
from abortState import AbortState
from moveRobotToNamedPose import MoveRobotToNamedPose
from publisherState import PublisherState
from getBinCoordinatesState import GetBinCoordinatesState
from toggleBinFillersAndTote import ToggleBinFillersAndTote
from setNextBin import SetNextBin
from getData import GetData
from baxter_core_msgs.msg import EndEffectorCommand
from scanShelfState import ScanShelfState
from getKinfuCloud import GetKinfuCloud
from shelfBasedCropCloud import ShelfBasedCropCloud
import waitState
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
        sm.add('deactivate_suction_init',
            PublisherState(topic='/robot/end_effector/right_gripper/command',
                    datatype=EndEffectorCommand,
                    data={'command':'stop', 'sender':'user', 'id': 65537},
                    n=5, hz=10),
            transitions={'succeeded':'set_the_next_bin'})


        sm.add('set_the_next_bin', SetNextBin(action='nothing'),
            transitions={'succeeded':'set_the_shelf_collision_scene',
             'failed': 'abort_state'})

        sm.add('set_the_shelf_collision_scene', ToggleBinFillersAndTote(action='fill_bins'),
            transitions={'succeeded':'set_the_tote_collision_scene',
             'failed': 'abort_state'})

        # add tote and tote pillar to collision scene
        # tote/pillar actions: tote_on, tote_off, pillar_on, pillar_off, all_on, all_off
        sm.add('set_the_tote_collision_scene', ToggleBinFillersAndTote(action='all_on'),
             transitions={'succeeded':'move_to_neutral_start',
             'failed': 'abort_state'})

        #move to a neutral start
        sm.add('move_to_neutral_start', MoveRobotToNamedPose(movegroup='left_arm',
                                    goal_pose_name='left_neutral'),
                transitions={'succeeded':'move_arm_to_look_into_bin',    # 'lift_object_up'
                             'failed': 'abort_state'})

        # move arm in front of bin before looking for the object
        sm.add('get_target_bin_coordinates', GetBinCoordinatesState(),
            transitions={'succeeded':'move_arm_to_look_into_bin','aborted':'abort_state'})
        #
        # sm.add('move_arm_to_look_into_bin', MoveRobotState(movegroup='left_arm_realsense'),
        sm.add('move_arm_to_look_into_bin', MoveRobotToNamedPose(movegroup='left_arm',
                                    name_prefix='look_into_'),
            transitions={'succeeded':'reset_kinfu',
                         'failed': 'abort_state',
                         'aborted':'abort_state'})

        sm.add('reset_kinfu',
             PublisherState(topic='/ros_kinfu/reset', datatype=Empty, data={}),
             transitions={'succeeded':'wait_sync1', 'aborted':'aborted'})

        sm.add('wait_sync1', waitState.WaitState(2.0),
             transitions={'succeeded':'get_kinfu_cloud'})


        # sm.add('toggle_lip_off', ToggleBinFillersAndTote(action='lip_off'),
        #  transitions={'succeeded':'shelf_scan',
        #   'failed': 'abort_state'})
        #
        # sm.add('shelf_scan',ScanShelfState(),
        #  transitions={'succeeded':'toggle_lip_on','failed':'abort_state'})
        #
        # sm.add('toggle_lip_on', ToggleBinFillersAndTote(action='lip_on'),
        #  transitions={'succeeded':'get_kinfu_cloud',
        #   'failed': 'abort_state'})

        sm.add('get_kinfu_cloud',GetKinfuCloud(),
         transitions={'succeeded':'crop_kinfu_cloud','failed':'abort_state'})

        sm.add('crop_kinfu_cloud',ShelfBasedCropCloud(),
        transitions={'succeeded':'get_data','failed':'reset_kinfu'})

        sm.add('get_data', GetData(),
            transitions={'succeeded':'set_the_next_bin',
                          'failed':'reset_kinfu'})

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
