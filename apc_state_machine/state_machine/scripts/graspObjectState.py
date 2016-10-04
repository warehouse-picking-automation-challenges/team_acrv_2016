import rospy
import smach
import moveit_lib.srv
from std_msgs.msg import String
from std_msgs.msg import Float32,Float64
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
from baxter_core_msgs.msg import EndEffectorCommand
import math
import tf
from tf import transformations
from suctionState import SuctionState
from copy import deepcopy
from std_srvs.srv import Trigger, TriggerRequest,SetBool,SetBoolRequest
from apc_msgs.srv import FillUnfillBinsCollisionModel, FillUnfillBinsCollisionModelRequest

class GraspObjectState(smach.State):
    def __init__(self, movegroup=None, velocity_scale=1.0):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted', 'failed'],
        input_keys=['next_item_to_pick','goal_frame_id','goal_pose_array','pre_grasp_pose_array','move_group_name_array'],
        output_keys=['goal_pose','goal_pose_array','move_group']
        )

        # if movegroup is None:
        #     rospy.logwarn("Parameter movegroup not specified. GraspObjectState instance will not execute.")

        # self.movegroup = movegroup
        self.velocity_scaling_factor = velocity_scale
        self.max_attempts = 1
        self.planning_time = 0.5

        # wait for the service to appear
        rospy.loginfo('Waiting for service move_robot_pose to come online ...')
        try:
            rospy.wait_for_service('/moveit_lib/move_robot_pose', timeout=0.1)
        except:
            rospy.logerr('Service %s not available. Restart and try again.'
                % '/moveit_lib/move_robot_pose')
          #  rospy.signal_shutdown('')

        # create a proxy to that service
        self.service = rospy.ServiceProxy(
            '/moveit_lib/move_robot_pose', moveit_lib.srv.move_robot_pose)
        self.cartesian_service = rospy.ServiceProxy(
            '/moveit_lib/move_robot_pose_array', moveit_lib.srv.move_robot_pose_array)
        self.named_service = rospy.ServiceProxy(
            '/moveit_lib/move_robot_named', moveit_lib.srv.move_robot_named)

        self.tf_listener = tf.TransformListener()

        self.pumpPublisher = rospy.Publisher('/robot/end_effector/right_gripper/command', EndEffectorCommand, queue_size = 1)
        self.valvePublisher = rospy.Publisher('/robot/end_effector/left_gripper/command', EndEffectorCommand, queue_size = 1)

    # ==========================================================
    def execute(self, userdata):
        # try to call the service
        # !! warning, this could potentially block forever
        try:
            self.suctionState = SuctionState()
            velocity_scale = Float32()
            velocity_scale.data = self.velocity_scaling_factor
            max_attempts = Int32()
            max_attempts.data = self.max_attempts
            planning_time = Float32()
            planning_time.data = self.planning_time
            #
            # if self.movegroup is None:
            #     rospy.logerr("Parameter movegroup is not specified. Aborting robot motion.")
            #     return 'aborted'

            # goal_frame_id = userdata['goal_frame_id']
            goal_pose_array = userdata['goal_pose_array']
            pre_grasp_pose_array = userdata['pre_grasp_pose_array']
            move_group_name_array = userdata['move_group_name_array']
            # rospy.loginfo('Trying to move robot\'s %s. Goal frame is %s' %
            #     (self.movegroup, goal_frame_id))
            # rospy.loginfo('goal pose: %s' % goal_pose)

            # print self.tf.lookupTransform(goal_frame_id, '/base', 0)
            # movegroup=String()
            # movegroup.data=self.movegroup

            for i in range(len(goal_pose_array.poses)):
                grasp_pose_stamped = PoseStamped()
                grasp_pose_stamped.pose = goal_pose_array.poses[i]

                grasp_pose_stamped.header.frame_id = goal_pose_array.header.frame_id
                grasp_pose_stamped.header.stamp = rospy.Time.now()

                pre_grasp_pose_stamped = PoseStamped()
                pre_grasp_pose_stamped.pose = pre_grasp_pose_array.poses[i]

                pre_grasp_pose_stamped.header.frame_id = goal_pose_array.header.frame_id
                pre_grasp_pose_stamped.header.stamp = rospy.Time.now()

                movegroup = String()
                movegroup.data = move_group_name_array[i]

                rospy.loginfo("[GraspState] Trying to move robot\'s '%s'." %
                    movegroup.data)

                # print '\n--------------------The pose is:'
                # print grasp_pose_stamped.pose
                # print '---------------------\n'
                # turn the valve to the correct sucker
                self.activateValve(movegroup)

                # TERRIBLE HACK TO HELP WITH THE WRONG SUCKER GRABBING ONTO THE WRONG ITEM
                rospy.sleep(1)

                # turn the pump on!
                self.suctionState.activatePump()


                # Use Standard Path
                print 'Attempting to move to pre grasp and grasp pose: ' #, pre_grasp_pose_stamped
                # Plan in joint space
                # move_to_pre_grasp_pose_resp = self.service.call(movegroup, pre_grasp_pose_stamped, velocity_scale, max_attempts,planning_time)
                # Plan in cartesian space
                cartesian_pose_array = PoseArray()
                cartesian_pose_array.header.frame_id = pre_grasp_pose_stamped.header.frame_id
                cartesian_pose_array.header.stamp = rospy.Time.now()
                cartesian_pose_array.poses.append(pre_grasp_pose_stamped.pose)
                cartesian_pose_array.poses.append(grasp_pose_stamped.pose)
                movegroup_cartesian = deepcopy(movegroup)
                movegroup_cartesian.data = movegroup_cartesian.data + '_cartesian'
                velocity_scale_grasp = Float32()
                velocity_scale_grasp.data = 1.0
                min_path_completion = Float32()
                min_path_completion.data = 0.25
                self.execute_unfill_bins()  # Turn off the shelf collision model
                move_in_resp = self.cartesian_service.call(movegroup_cartesian, cartesian_pose_array, velocity_scale_grasp, min_path_completion, max_attempts, planning_time)

                # if(move_to_pre_grasp_pose_resp.success.data == True):
                #     print 'Attempting to grasp object at pose: ' #, grasp_pose_stamped
                #     # response = self.service.call(movegroup, grasp_pose_stamped, velocity_scale, max_attempts,planning_time)
                #
                #
                #
                #     # Use Cartesian Path - need to add in check for 100% complete though
                #     cartesian_pose_array = PoseArray()
                #     cartesian_pose_array.header.frame_id = grasp_pose_stamped.header.frame_id
                #     cartesian_pose_array.header.stamp = rospy.Time.now()
                #     cartesian_pose_array.poses.append(grasp_pose_stamped.pose)
                #     movegroup_cartesian = deepcopy(movegroup)
                #     movegroup_cartesian.data = movegroup_cartesian.data + '_cartesian'
                #     velocity_scale_grasp = Float32()
                #     velocity_scale_grasp.data = 1.0
                #     # move_in_resp = self.cartesian_service.call(movegroup_cartesian, cartesian_pose_array, velocity_scale,min_path_completion)
                #     move_in_resp = self.cartesian_service.call(movegroup_cartesian, cartesian_pose_array, velocity_scale_grasp,min_path_completion)

                if move_in_resp.success.data == True:
                    # Fill in goal_pose with the pick pose that worked so that next up relative move works
                    userdata['goal_pose'] = grasp_pose_stamped
                    userdata['goal_pose_array'] = []
                    userdata['move_group'] = movegroup.data  ## To keep consistent with main state maching interface
                    return 'succeeded'
                else:
                    self.suctionState.deactivatePump()

        except Exception, e:
            print e
            rospy.logerr('Service call to move_robot_pose failed. Reason: \n %s' % e)
            return 'aborted'

        # if move_in_resp.success.data == True:
        #     return 'succeeded'

        return 'failed'



    def activateValve(self, movegroup):
        print "\n\n\n\n\n\Valve: "
        # move valve, select the correct pipe
        # Activate suction for correct move group
        # the valve is connected to the left gripper outlet
        if movegroup.data == 'left_arm_cartesian' or movegroup.data == 'left_arm' or movegroup.data == 'right_arm'  or movegroup.data == 'right_arm_cartesian':
            print "straight go"
            msg = EndEffectorCommand()
            msg.command = 'stop'
            msg.id = 65537
            msg.sender = 'user'
            msg.args = "{\"grip_attempt_seconds\": 1000}"
            self.valvePublisher.publish(msg)
            rospy.loginfo('Left')

        if movegroup.data == 'left_arm_90_cartesian' or movegroup.data == 'left_arm_90' or movegroup.data == 'right_arm_90_cartesian' or movegroup.data == 'right_arm_90':
            # NOTE must trigger line low before high to guarantee success
            msg = EndEffectorCommand()
            msg.command = 'stop'
            msg.id = 65537
            msg.sender = 'user'
            self.valvePublisher.publish(msg)

            print "90 go"
            msg = EndEffectorCommand()
            msg.command = 'go'
            msg.id = 65537
            msg.sender = 'user'
            msg.args = "{\"grip_attempt_seconds\": 1000}"
            self.valvePublisher.publish(msg)

    def execute_unfill_bins(self):
        # try to call the service
        srv_name = '/apc_grasping/unfill_bins'
        self.service = rospy.ServiceProxy(srv_name, Trigger)
        try:
            req = TriggerRequest()
            response = self.service.call(req)
            rospy.loginfo('All bins have been unfilled')
            return 'succeeded'
        except Exception,e:
            rospy.logerr('Service call to unfill_bins failed. Reason: \n %s' % e)
            return 'failed'
