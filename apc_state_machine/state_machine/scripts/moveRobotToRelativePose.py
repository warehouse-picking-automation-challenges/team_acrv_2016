import rospy
import smach
import moveit_lib.srv
import numpy as np
import baxter_interface
from std_msgs.msg import String
from std_msgs.msg import Float32, Int32
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
from copy import deepcopy

import tf

class MoveRobotToRelativePose(smach.State):
    # send either a prefix to be added to (usually) the BIN name from the
    def __init__(self, relative_pose=None, pose_frame_id='',
                 movegroup=None, velocity_scale=1.0):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted', 'failed'])

        if movegroup is None:
            rospy.logwarn("Parameter movegroup not specified. MoveRobotToRelativePose instance will not execute.")

        self.relative_pose_ = relative_pose
        self.pose_frame_id_ = pose_frame_id
        self.movegroup_ = movegroup
        self.velocity_scaling_factor = velocity_scale

        # that's the cartesian service implementation
        service_name = '/moveit_lib/move_robot_pose_array'
        # wait for the service to appear
        rospy.loginfo('Waiting for service %s to come online ...' % service_name)
        try:
            rospy.wait_for_service(service_name, timeout=0.1)
        except:
            rospy.logerr('Service %s not available. Restart and try again.' % service_name)

        self.service_ = rospy.ServiceProxy(service_name,
                                    moveit_lib.srv.move_robot_pose_array)

        self.tf_listener = tf.TransformListener()

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
        try:
            rospy.loginfo('Trying to move robot\'s %s to a relative pose.' %
                            self.movegroup_)

            if self.movegroup_ is None:
                rospy.logerr("Parameter movegroup is not specified. Aborting robot motion.")
                return 'aborted'

            frame_id = self.pose_frame_id_
            # copy the pose!!!!!!!!!
            goal_pose = deepcopy(self.relative_pose_)
            if type(goal_pose) == Pose:
                goal_pose_stamped = PoseStamped()
                goal_pose_stamped.pose = goal_pose
                goal_pose_stamped.header.frame_id = frame_id
                goal_pose_stamped.header.stamp = rospy.Time.now()
            elif type(goal_pose) == PoseStamped:
                goal_pose_stamped = goal_pose
            else:
                rospy.logerr('Unknown data type in MoveRobotToRelativePose must either be PoseStamped or Pose')
                return 'aborted'

            rospy.loginfo('Trying to move robot\'s %s. Relative to frame %s' %
                (self.movegroup_, frame_id))
            rospy.loginfo('goal pose: %s' % goal_pose_stamped)

            movegroup = String()
            movegroup.data = self.movegroup_
            velocity_scale = Float32()
            velocity_scale.data = self.velocity_scaling_factor

            # get pose of left or right hand in the specified target frame
            if self.movegroup_ == 'left_arm' or self.movegroup_ == 'left_arm_cartesian':
                hand_frame = '/left_endpoint'
            elif self.movegroup_=='left_arm_90' or self.movegroup_ == 'left_arm_90_cartesian':
                hand_frame = '/left_endpoint_90'
            elif self.movegroup_ == 'right_arm' or self.movegroup_ == 'right_arm_cartesian':
                hand_frame = '/right_endpoint'
            elif self.movegroup_=='right_arm_90' or self.movegroup_ == 'right_arm_90_cartesian':
                hand_frame = '/right_endpoint_90'
            else:
                rospy.logerr('Unknown movegroup: %s' % self.movegroup_)
                return 'aborted'

            # painful tf lookup!
            # rospy.loginfo('Looking up transform from %s into %s frames....' % (frame_id, hand_frame))
            self.tf_listener.waitForTransform(frame_id, hand_frame, rospy.Time(0), rospy.Duration(3.0))

            common_time = self.tf_listener.getLatestCommonTime(frame_id, hand_frame)

            # this creates the pose of the hand, expressed in the goal frame
            hand_in_goal_frame = self.tf_listener.lookupTransform(frame_id, hand_frame, common_time)
            # rospy.Time(0))

            # if self.tf_listener.frameExists(frame_id) and self.tf_listener.frameExists(hand_frame):
            #     common_time = self.tf_listener.getLatestCommonTime(frame_id, hand_frame)
            #     hand_in_goal_frame = self.tf_listener.lookupTransform(frame_id, hand_frame, common_time)

            print 'hand_in_goal_frame = '
            print hand_in_goal_frame

            # translate / rotate by the given pose
            # add requested translation component
            goal_pose_stamped.pose.position.x += hand_in_goal_frame[0][0]
            goal_pose_stamped.pose.position.y += hand_in_goal_frame[0][1]
            goal_pose_stamped.pose.position.z += hand_in_goal_frame[0][2]

            goal_pose_stamped.pose.orientation.x = hand_in_goal_frame[1][0]
            goal_pose_stamped.pose.orientation.y = hand_in_goal_frame[1][1]
            goal_pose_stamped.pose.orientation.z = hand_in_goal_frame[1][2]
            goal_pose_stamped.pose.orientation.w = hand_in_goal_frame[1][3]

            # TODO: implement rotation!  use hand_in_goal_frame[1][0-3]
            rospy.logwarn('Relative motion requested. Be aware that relative rotations are not yet implemented.')

            goal_pose_array = PoseArray()
            goal_pose_array.header.frame_id = frame_id
            goal_pose_array.header.stamp = rospy.Time.now()
            goal_pose_array.poses.append(goal_pose_stamped.pose)

            print "--------------------------"
            print "Trying to move to: ",
            print goal_pose_array.poses


            print 'calling move robot relative (cartesian) service'
            min_path_completion = Float32()
            min_path_completion.data = 0.25
            # added to the service call
            max_attempts = Int32()
            max_attempts.data = 30
            max_planning_time = Float32()
            max_planning_time.data = 7.0
            # trying to move to a joint position directly!
            rospy.loginfo("calling the move to named pose with %d attempts and %f sec", max_attempts.data, max_planning_time.data)
            flag = self.service_.call(movegroup, goal_pose_array, velocity_scale, min_path_completion, max_attempts, max_planning_time)

            if flag.success.data == True:
                return 'succeeded'

        except Exception, e:
            rospy.logerr('Cannot lookup a transform from %s to %s now, but required to perform relative motion of the %s move group.'  % (hand_frame, frame_id, self.movegroup_))
            print e
            return 'aborted'

        return 'failed'
