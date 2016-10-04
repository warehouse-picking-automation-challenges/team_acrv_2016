import rospy
import smach
import moveit_lib.srv
from std_msgs.msg import String
from std_msgs.msg import Float32
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
import tf

class MoveRobotState(smach.State):
    def __init__(self, goal_frame_id=None, named=False, name_prefix='',
                 goal_pose=None, movegroup=None, relative_motion=False,
                 velocity_scale=1.0):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted', 'failed'],
        input_keys=['next_item_to_pick','goal_frame_id','goal_pose','move_group']
        )

        if movegroup is None:
            rospy.logwarn("Parameter movegroup not specified. MoveRobotState instance will not execute.")

        if True == named or True == relative_motion:
            rospy.logerr('Not implemented here anymore!')
            raise Exception('not implmented anymore')

        self.named = named
        self.name_prefix = name_prefix
        self.goal_frame_id = goal_frame_id
        self.goal_pose = goal_pose
        if movegroup is not None:
            self.movegroup = movegroup
        self.relative_motion = relative_motion
        self.velocity_scaling_factor = velocity_scale
        self.max_attempts = Int32()
        self.max_attempts.data = 5
        self.max_planning_time = Int32()
        self.max_planning_time.data = 5

        # wait for the service to appear
        rospy.loginfo('Waiting for service move robot services to come online ...')
        try:
            rospy.wait_for_service('/moveit_lib/move_robot_pose', timeout=0.1)
        except:
            rospy.logerr('Service /moveit_lib/move_robot_pose not available. Restart and try again.')
          #  rospy.signal_shutdown('')

        # create a proxy to that service
        self.service = rospy.ServiceProxy(
            '/moveit_lib/move_robot_pose', moveit_lib.srv.move_robot_pose)
        self.cartesian_service = rospy.ServiceProxy(
            '/moveit_lib/move_robot_pose_array', moveit_lib.srv.move_robot_pose_array)
        self.named_service = rospy.ServiceProxy(
            '/moveit_lib/move_robot_named', moveit_lib.srv.move_robot_named)

        self.tf_listener = tf.TransformListener()

    # ==========================================================
    def execute(self, userdata):

        if 'move_group' in userdata:
            if userdata['move_group'] is not None:
                self.movegroup = userdata['move_group']

        # try to call the service
        # !! warning, this could potentially block forever
        try:

            velocity_scale = Float32()
            velocity_scale.data = self.velocity_scaling_factor

            if self.movegroup is None:
                rospy.logerr("Parameter movegroup is not specified. Aborting robot motion.")
                return 'aborted'

            if self.named == True:
                goal_pose = String()     # needs to mapt to something in from srdf file!
                if self.goal_frame_id == None:
                    goal_pose.data = self.name_prefix + userdata['next_item_to_pick']['bin']
                else:
                    goal_pose.data = self.name_prefix + self.goal_frame_id

                goal_frame_id = self.goal_frame_id
            elif self.goal_frame_id is None and self.goal_pose is None:
                goal_frame_id = userdata['goal_frame_id']
                goal_pose = userdata['goal_pose']
            elif self.goal_frame_id is None:
                goal_frame_id = userdata['goal_frame_id']
                goal_pose = self.goal_pose
            elif self.goal_pose is None:
                goal_frame_id = self.goal_frame_id
                goal_pose = userdata['goal_pose']
            else:
                goal_pose = self.goal_pose
                goal_frame_id = self.goal_frame_id

            rospy.loginfo('Trying to move robot\'s %s. Goal frame is %s' %
                (self.movegroup, goal_frame_id))
            #rospy.loginfo('goal pose: %s' % goal_pose)

            # print self.tf.lookupTransform(goal_frame_id, '/base', 0)
            movegroup=String()
            movegroup.data=self.movegroup

            if self.named: #type(goal_pose) == String:
                # trying to move to a joint position directly!\
                flag = self.named_service.call(movegroup, goal_pose, velocity_scale)
                if flag.success.data == True:
                    return 'succeeded'
                else:
                    return 'failed'
            elif type(goal_pose) == Pose:
                goal_pose_stamped = PoseStamped()
                goal_pose_stamped.pose = goal_pose
                goal_pose_stamped.header.frame_id = goal_frame_id
                goal_pose_stamped.header.stamp = rospy.Time.now()
            elif type(goal_pose) == PoseStamped:
                goal_pose_stamped = goal_pose
            else:
                rospy.logerr('Unknown data type in MoveRobotState. Must either be PoseStamped or Pose or String')
                return 'aborted'


            # if we are requested to move RELATIVE to the specified frame, we need some more calculations
            if self.relative_motion:
                print 'moving relative to frame: ', goal_frame_id
                # get pose of left or right hand in the specified target frame
                if self.movegroup=='left_arm':
                    hand_frame = '/left_endpoint'
                elif self.movegroup=='left_arm_90':
                    hand_frame = '/left_endpoint_90'
                    # hand_frame = '/left_gripper'
                # elif self.movegroup=='right_arm':
                #     hand_frame = '/right_gripper'
                else:
                    rospy.logerr('Unknown movegroup: %s' % self.movegroup)
                    return 'aborted'

                # painful tf lookup!
                try:
                    now = rospy.Time.now()
                    print 'waiting for tf...'
                    print 'hand_frame = ', hand_frame
                    print 'goal_frame_id = ', goal_frame_id
                    self.tf_listener.waitForTransform(goal_frame_id, hand_frame, now, rospy.Duration(4.0))
                    common_time = self.tf_listener.getLatestCommonTime(goal_frame_id, hand_frame)

                    # this is the pose of the hand, expressed in the goal frame
                    hand_in_goal_frame = self.tf_listener.lookupTransform(goal_frame_id, hand_frame, common_time)
                    # rospy.Time(0))
                    print 'hand_in_goal_frame = '
                    print hand_in_goal_frame

                except Exception, e:
                    rospy.logerr('Cannot lookup a transform from %s to %s now, but required to perform relative motion of the %s move group.'  % (hand_frame, goal_frame_id, self.movegroup))
                    print e
                    return 'aborted'

                # translate / rotate by the given pose
                # add requested translation component
                #
                goal_pose_stamped.pose.position.x += hand_in_goal_frame[0][0]
                goal_pose_stamped.pose.position.y += hand_in_goal_frame[0][1]
                goal_pose_stamped.pose.position.z += hand_in_goal_frame[0][2]

                goal_pose_stamped.pose.orientation.x = hand_in_goal_frame[1][0]
                goal_pose_stamped.pose.orientation.y = hand_in_goal_frame[1][1]
                goal_pose_stamped.pose.orientation.z = hand_in_goal_frame[1][2]
                goal_pose_stamped.pose.orientation.w = hand_in_goal_frame[1][3]

                # goal_pose_stamped.pose.position.x += 0
                # goal_pose_stamped.pose.position.y += 0
                # goal_pose_stamped.pose.position.z += 0.05

                goal_pose_array = PoseArray()
                goal_pose_array.header.frame_id = goal_frame_id
                goal_pose_array.header.stamp = rospy.Time.now()
                goal_pose_array.poses.append(goal_pose_stamped.pose)

                # TODO: implement rotation!  use hand_in_goal_frame[1][0-3]
                rospy.logdebug('Relative motion requested. Be aware that relative rotations are not yet implemented.')


                print 'calling move robot cartesian_service'
                print 'yoooooooo'
                print 'movegroup = ', movegroup
                print 'frame_id = ', goal_pose_array
                flag = self.cartesian_service.call(movegroup, goal_pose_array, velocity_scale)
                # print 'calling move robot'
                # flag = self.service.call(movegroup, goal_pose_stamped, velocity_scale, self.max_attempts, self.max_planning_time)
            else:
                print 'calling move robot stamped', goal_pose_stamped
                flag = self.service.call(movegroup, goal_pose_stamped, velocity_scale, self.max_attempts, self.max_planning_time)

        except Exception, e:
            if self.named == True:
                rospy.logerr('Some Exception in MoveRobotState NAMED.')
                print e
            else:
                rospy.logerr('Service call to move_robot_pose failed. Reason: \n %s' % e)
            return 'aborted'

        if flag.success.data == True:
            return 'succeeded'

        return 'failed'
