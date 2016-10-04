#! /usr/bin/env python

import roslib
import rospy
import smach
import smach_ros

import threading

from findObject2DState import FindObject2DState
from decideGraspPoseState import DecideGraspPoseState
from moveRobotState import MoveRobotState
from decideNextPickItemState import DecideNextPickItemState
from popItemState import PopItemState
from pickSuccessfulState import PickSuccessfulState

from getBinCoordinatesState import GetBinCoordinatesState
from publisherState import PublisherState
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
    rospy.init_node('smach_shelf_picking')

    # Create the top level SMACH state machine
    sm = smach.StateMachine(outcomes=['succeeded', 'aborted', 'preempted'])

    # create states and connect them
    with sm:

        # decide which object to grasp next from which bin
        sm.add('decide_next_pick', DecideNextPickItemState(),
            transitions={'succeeded':'get_target_bin_coordinates', 'finished': 'succeeded'})

        # sm.add('wait', waitState.WaitState(1.0),
        #     transitions={'succeeded':'remove_current_pick'})

        # move arm in front of bin before looking for the object
        sm.add('get_target_bin_coordinates', GetBinCoordinatesState(),
            transitions={'succeeded':'move_arm_to_bin','aborted':'decide_next_pick'})

        sm.add('move_arm_to_bin', MoveRobotState(),
            transitions={'succeeded':'wait_sync1', 'aborted':'object_detection_2D'})

        sm.add('wait_sync1', waitState.WaitState(0.0),
            transitions={'succeeded':'object_detection_2D'})

        sm.add('object_detection_2D', FindObject2DState(),
            transitions={'no_objects':'decide_next_pick', 'succeeded':'decide_grasp_pose'})

        sm.add('decide_grasp_pose', DecideGraspPoseState(),
            transitions={'succeeded':'activate_suction', 'failed':'move_arm_to_bin'})

        # activate suction here before moving the arm
        sm.add('activate_suction',
            PublisherState(topic='/robot/end_effector/right_gripper/command',
                    datatype=EndEffectorCommand,
                    # data={'command':'go', 'sender':'user', 'id': 65537, 'args':'{\"grip_attempt_seconds\": 100}'},
                    data={'command':'go', 'sender':'user', 'id': 65538, 'args':'{\"grip_attempt_seconds\": 100}'},
                    n=5, hz=10),
            transitions={'succeeded':'grasp_object_in_bin', 'aborted':'decide_next_pick'})

        # move the sucker into the object, colliding with it slightly
        sm.add('grasp_object_in_bin', MoveRobotState(),
             transitions={'succeeded':'wait_sucking', 'aborted':'decide_next_pick'})

        sm.add('wait_sucking', waitState.WaitState(0.0),
             transitions={'succeeded':'lift_object_from_bin'})

        # lift the object up a little bit
        sm.add('lift_object_from_bin', MoveRobotState(goal_frame_id='/shelf', goal_pose=Pose(position=Point(-0.05,0,0), orientation=Quaternion(0,0,0,1)), relative_motion=True),
             transitions={'succeeded':'remove_object_from_bin'})

        # sm.add('wait', waitState.WaitState(10.0),
        #      transitions={'succeeded':'remove_object_from_bin'})

        # retreat from the bin in a straight line
        sm.add('remove_object_from_bin', MoveRobotState(goal_frame_id='/shelf', goal_pose=Pose(position=Point(0,0,-0.45), orientation=Quaternion(0,0,0,1)), relative_motion=True),
             transitions={'succeeded':'move_to_drop_pose'})


        # move the object on top of the tote to drop it
        sm.add('move_to_drop_pose', MoveRobotState(goal_frame_id='/base', goal_pose=Pose(position=Point(0.4,-0.9,-0.1), orientation=Quaternion(0,1,0,0)), relative_motion=False),
             transitions={'succeeded':'deactivate_suction'})

        # deactivate suction to let go of the object
        sm.add('deactivate_suction',
            PublisherState(topic='/robot/end_effector/right_gripper/command',
                    datatype=EndEffectorCommand,
                    # data={'command':'stop', 'sender':'user', 'id': 65537},
                    data={'command':'stop', 'sender':'user', 'id': 65538},
                    n=5, hz=10),
            transitions={'succeeded':'pick_successful'})

        sm.add('pick_successful', PickSuccessfulState(),
            transitions={'succeeded':'decide_next_pick'})






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
