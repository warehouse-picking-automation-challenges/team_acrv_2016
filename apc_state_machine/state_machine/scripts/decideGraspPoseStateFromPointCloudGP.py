import rospy
import smach

import apc_msgs.srv
from sensor_msgs.msg import PointCloud2

import time
import json


class DecideGraspPoseStateFromPointCloud(smach.State):
    def __init__(self,action='pick_item'):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'],
            input_keys=['next_item_to_pick','object_proposals','googlenet','intelligent_pick','current_item_attempts'],
            output_keys=['goal_pose', 'goal_pose_array', 'pre_grasp_pose_array', 'move_group_name_array']
        )

        # read the item-specific move group names filename from the ROS parameter server
        try:
            moveGroupsFileName = rospy.get_param('/item_move_groups')
        except:
            rospy.logerr('Required parameter mission_file not found.')
            moveGroupsFileName = None

        try:
            current_task = rospy.get_param('/tell_me_the_task_again_please')
            print 'The current task (if you needed reminding) is: ', current_task, '\n'
        except:
            rospy.logerr('Required parameter tell_me_the_task_again_please not found.')
            current_task = None

        self.current_task = current_task

        if self.current_task == "stow":
            self.is_grasp_vertical_only = True
        else:
            self.is_grasp_vertical_only = False

        # read the move groups from the provided json file
        self.potentialMoveGroups = self.readJSON(moveGroupsFileName)

        self.action = action
        # wait for the services to appear
        self.waitForService('/apc_grasping/detect_grasp_candidates')
        self.waitForService('/apc_grasping/grasp_selection_from_candidates')

        # create a proxy to that service
        self.srv_graspCandidates = rospy.ServiceProxy('/apc_grasping/detect_grasp_candidates', apc_msgs.srv.DetectGraspCandidates)
        self.srv_selectGraspFromCandidates = rospy.ServiceProxy('/apc_grasping/grasp_selection_from_candidates', apc_msgs.srv.SelectGraspFromCandidates)

        self.points_pub = rospy.Publisher('pointsBeforeChris', PointCloud2, queue_size=10)

    # ==========================================================
    def waitForService(self, service):

        rospy.loginfo('Waiting for service %s to come online ...' % service)
        try:
            rospy.wait_for_service(service, timeout=0.1)
        except:
            rospy.logerr('Service %s not available. Restart and try again.' % service)
            return False
        else:
            return True

    # =========================================================================
    def readJSON(self, filename):
        try:
            return json.load(file(filename, 'r'))
        except:
            rospy.logerr('Fatal Error: Could not read mission description from file %s' % filename)
            return None

    # ==========================================================
    def getPreSelectedPotentialMoveGroups(self, item_name):
        return self.potentialMoveGroups[item_name]

    # ==========================================================
    def execute(self, userdata):

        ## IN THE CASE OF FAILED GRASPING, THIS MIGHT BE USEFUL FOR SELECTING NEXT GRASP POSE
        # if 'goal_pose_array' is not in userdata.keys():

        if self.action is 'pick_item':
            segment = userdata['googlenet']['item_segment']
        else:
            # TODO INTELLIGENT SELECTION REQUIED BEFORE THIS STATE, can probably do away with action after this
            segment = userdata['intelligent_pick']['segment']

        pointcloud_segment = segment

        # call Chris' grasp pose service with that pointcloud
        rospy.loginfo('Calling grasp point detection service ... ')
        self.points_pub.publish(pointcloud_segment)
        if(userdata['current_item_attempts'] < 2):  # after 0 failed attempts, use centroid only
            grasp_candidates_response = self.srv_graspCandidates.call(pointcloud_segment, 0.01, False)
        else:
            grasp_candidates_response = self.srv_graspCandidates.call(pointcloud_segment, 0.01, True) # True for grasping centroid only

        if len(grasp_candidates_response.grasp_candidates.grasp_utilities) == 0:
            rospy.logwarn("Could not find any suitable grasp pose!")
            return 'failed'
        else:
            bin_name = userdata['next_item_to_pick']['bin']
            item_move_groups = self.getPreSelectedPotentialMoveGroups(userdata['next_item_to_pick']['item'])

            if(userdata['current_item_attempts'] < 2):  # after 0 failed attempts, use centroid only
                grasp_selection_response = self.srv_selectGraspFromCandidates.call(
                            grasp_candidates_response.grasp_candidates,
                            bin_name, item_move_groups, False)
            else:
                grasp_selection_response = self.srv_selectGraspFromCandidates.call(
                            grasp_candidates_response.grasp_candidates,
                            bin_name, item_move_groups, True)

            if grasp_selection_response.success.data == False:
                print 'Grasp selection found no reachable grasps :(\n'
                return 'failed'

            # store it for the other states in the state machine
            userdata['goal_pose'] = grasp_selection_response.selected_grasps.poses[0]
            userdata['goal_pose_array'] = grasp_selection_response.selected_grasps
            userdata['pre_grasp_pose_array'] = grasp_selection_response.selected_pre_grasps
            userdata['move_group_name_array'] = grasp_selection_response.grasp_move_group

            return 'succeeded'

        ## IN THE CASE OF FAILED GRASPING, THIS MIGHT BE USEFUL FOR SELECTING NEXT GRASP POSE
        # else:
        #     userdata['goal_pose_array'].poses.pop(0)
        #     userdata['pre_grasp_pose_array'].poses.pop(0)
        #     userdata['move_group_name_array'].pop(0)
        #     userdata['goal_pose'] = userdata['goal_pose_array'][0]
