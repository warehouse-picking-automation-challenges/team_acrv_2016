import rospy
import smach
import moveit_lib.srv
import baxter_interface

import numpy as np


import time

from std_msgs.msg import String
from std_msgs.msg import Int32, Float32

class MoveRobotToNamedPose(smach.State):
    # send either a prefix to be added to (usually) the BIN name from the
    def __init__(self, name_prefix='', goal_pose_name=None, movegroup=None,
                 velocity_scale=1.0):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'],
                                   input_keys=['next_item_to_pick'])

        if movegroup is None:
            rospy.logwarn("Parameter movegroup not specified. MoveRobotToNamedPose instance will not execute.")

        self.named = True
        self.name_prefix = name_prefix
        self.goal_pose_name = goal_pose_name
        self.movegroup = movegroup
        self.velocity_scaling_factor = velocity_scale

        service_name = '/moveit_lib/move_robot_named'
        # wait for the service to appear
        rospy.loginfo('Waiting for service %s to come online ...' % service_name)
        try:
            rospy.wait_for_service(service_name, timeout=0.1)
        except:
            rospy.logerr('Service %s not available. Restart and try again.' % service_name)

        self.service_ = rospy.ServiceProxy(service_name,
                                    moveit_lib.srv.move_robot_named)



    # ================================================================
    # from baxter examples
    def jitter(self):
        rospy.loginfo("jitter around a bit!")
        limb = baxter_interface.Limb('left')
        # joint_name = limb.joint_names()[5]
        delta = 0.05  # in rad
        # current_position = limb.joint_angle(joint_name)
        # joint_command = { joint_name: current_position + delta }
        values = [ limb.joint_angle(joint_name) + ((np.random.rand() * delta) - delta/2.0)  for joint_name in limb.joint_names() ]
        # print values
        joint_command = dict([(joint, values[i]) for i, joint in enumerate(limb.joint_names()) ])
        # print joint_command
        limb.set_joint_positions(joint_command)

        rospy.loginfo("jittered!")


    # ==========================================================
    def execute(self, userdata):
        # try to call the service
        # !! warning, this could potentially block forever
        try:
            if self.movegroup is None:
                rospy.logerr("Parameter movegroup is not specified. Aborting robot motion.")
                return 'failed'

            goal_pose = String()     # needs to mapt to something in from srdf file!
            if self.goal_pose_name == None:
                goal_pose.data = self.name_prefix + userdata['next_item_to_pick']['bin']
            else:
                goal_pose.data = self.name_prefix + self.goal_pose_name

            rospy.loginfo('Trying to move robot\'s %s. Goal is named pose: %s' %
                (self.movegroup, goal_pose.data))

            # rospy.loginfo('goal pose: %s' % goal_pose)
            # print self.tf.lookupTransfor
            movegroup = String()
            movegroup.data = self.movegroup
            velocity_scale = Float32()
            velocity_scale.data = self.velocity_scaling_factor

            # added to the service call
            max_attempts = Int32()
            max_attempts.data = 3
            max_planning_time = Float32()
            max_planning_time.data = 5.0

            # trying to move to a joint position directly!
            rospy.loginfo("calling the move to named pose with %d attempts and %f sec", max_attempts.data, max_planning_time.data)
            print movegroup, goal_pose, velocity_scale
            flag = self.service_.call(movegroup, goal_pose, velocity_scale, max_attempts, max_planning_time)
            print "blah"
            # if it fails do the jitter
            n_jitters = 0
            max_jitters = 3

            while flag.success.data == False and n_jitters < max_jitters:
                self.jitter()
                rospy.loginfo("again calling the move to named pose with %d attempts and %f sec", max_attempts.data, max_planning_time.data)
                flag = self.service_.call(movegroup, goal_pose, velocity_scale, max_attempts, max_planning_time)
                n_jitters += 1

        except Exception, e:
            rospy.logerr('Some Exception in MoveRobotToNamedPose!')
            print e
            return 'failed'

        if flag.success.data == True:
            return 'succeeded'

        return 'failed'
