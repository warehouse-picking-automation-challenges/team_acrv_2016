import rospy
import smach
import smach_ros
import threading
import tf
from tf import transformations
import math
import yaml

from geometry_msgs.msg import PoseStamped

class DecideGraspPoseState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'],
            input_keys=['next_item_to_pick','goal_frame_id', 'goal_pose'],
            output_keys=['next_item_to_pick','goal_frame_id', 'goal_pose']
            )

        # define object dependent optimal grasp poses
        # just use (0,0,0) for the while being
        self.graspPoses={}
        optimalPose=[0,0,0]
        for i in range(10):
            self.graspPoses['object_%d' % i ] = optimalPose

    # ==========================================================
    def execute(self, userdata):
        # not much going on here at the moment, might get more sophisticated in the future
        # just fill in the target frame id and desired pose (0,0,0)

        target_pose = PoseStamped()

        # here we would look at userdata['next_item_to_pick']['item'] and look up the
        # 2D objects that are associated with it.
        # Using the information of the other found objects, we can determine which view is less
        # occluded by other objects.
        # Then look at the normal vectors of the associated coordinate frames and
        # choose the one that is most perpendicular to the shelf frame (i.e. the object
        # is most parallel to the shelf front = most easy to reach)

        # for now, we don't do anything, but simply always go for the first object in the list
        userdata['goal_frame_id']
        target_pose = userdata['goal_pose']

        # we have to rotate by 180 deg around the Z axis and -90 around the Y axis to match
        # the orientation of the object coord frame to the hand frame
        # q = transformations.quaternion_from_euler(0, - math.pi / 2.0, math.pi, 'rxyz')
        # target_pose.pose.orientation.x = q[0]
        # target_pose.pose.orientation.y = q[1]
        # target_pose.pose.orientation.z = q[2]
        # target_pose.pose.orientation.w = q[3]

        # just changing to move "into" the object
        # target_pose.position.y +=  0.01
        # target_pose.position.x += -0.04

        target_pose.position.x -= 0.01  # TODO make this a parameter

        userdata['goal_pose'] = target_pose

        rospy.loginfo('Decided to grasp at %s' % target_pose)
        return 'succeeded'


    # ==========================================================
    def execute2(self, userdata):
        # not much going on here at the moment, might get more sophisticated in the future
        # just fill in the target frame id and desired pose (0,0,0)

        while(1):

            target_pose = PoseStamped()
            tf_listener = tf.TransformListener()

            # are we looking for most perpendicular to the shelf frame or the gripper?
            # we can compare to the shelf if we have a shelf tf.
            # assuming id_to_obj dictionary.
            with open('id_to_obj.yaml', 'r') as f:
                    id_to_obj = yaml.load(f)

            next_item_ids = [];
            next_item_dot = [];

            for i in userdata['found_2d_objects']:
                    if (id_to_obj[i] == userdata['next_item_to_pick']['item']):
                            next_item_ids.append(i)
            if not next_item_ids:
                    return 'failed'

            for i in next_item_ids:
                    if self.tf.frameExists('object_%d' % i) and self.tf.frameExists('/left_gripper'):
                            t = self.tf.getLatestCommonTime('object_%d' % i, '/left_gripper')
                            position, quaternion = self.tf.lookupTransform('object_%d' % i, '/left_gripper', t)

                            euler = transformations.euler_from_quaternion(quaternion)
                            euler[1] -= math.pi / 2.0
                            euler[2] += math.pi

                            dot = sqrt((euler[0]/(math.pi/2.0))^2 + (euler[1]/(math.pi/2.0))^2)
                            # assuming z is into, x is up, y is right
                            next_item_dot.append(dot)

            i = next_item_dot.index(min(values))

            frame_id = 'object_%d' % next_item_ids[i]
            userdata['goal_frame_id'] = frame_id


            # DEBUG #
            print userdata['next_item_to_pick']['item']
            print next_item_ids
            print id_to_obj
            print next_item_dot
            print 'object_%d' % next_item_ids[i]



            target_pose.header.frame_id = frame_id
            target_pose.header.stamp = rospy.Time.now()


            # if target_pose is the gripper position from 'goal_frame_id' :
            q = transformations.quaternion_from_euler(0, - math.pi / 2.0, math.pi, 'rxyz')
            target_pose.pose.orientation.x = q[0]
            target_pose.pose.orientation.y = q[1]
            target_pose.pose.orientation.z = q[2]
            target_pose.pose.orientation.w = q[3]
            target_pose.pose.position.x = -0.06

            # # if target_pose is the gripper position from its current position :
            #
            # if self.tf.frameExists('object_%d' % i) and self.tf.frameExists('/left_gripper'):
            #         t = self.tf.getLatestCommonTime('object_%d' % next_item_ids[i], '/left_gripper')
            #         position, quaternion = self.tf.lookupTransform('object_%d' % i, '/left_gripper', t) # bad scope
            # euler = transformations.euler_from_quaternion(quaternion)
            # euler[1] -= math.pi / 2.0
            # euler[2] += math.pi
            #
            # q = transformations.quaternion_from_euler(euler[0], euler[1], euler[2], 'rxyz')
            # target_pose.pose.orientation.x = q[0]
            # target_pose.pose.orientation.y = q[1]
            # target_pose.pose.orientation.z = q[2]
            # target_pose.pose.orientation.w = q[3]
            # target_pose.pose.position.x = position[0]
            # target_pose.pose.position.y = position[1]
            # target_pose.pose.position.z = position[2]
            #
            # print target_pose.pose.position
            # print euler

            userdata['goal_pose'] = target_pose

            rospy.loginfo('Decided to grasp at %s' % target_pose)

        return 'succeeded'
