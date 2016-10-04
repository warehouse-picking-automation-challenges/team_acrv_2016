import rospy
import smach
import smach_ros

import message_filters

import tf
from tf import transformations
from tf import TransformListener
from tf import transformations

from geometry_msgs.msg import PoseStamped

import apc_msgs.srv
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import Image
from apc_msgs.msg import PointcloudRGB
from apc_msgs.msg import ObjectProposals
from apc_msgs.msg import GraspCandidates
from apc_msgs.srv import DoSegmentation,DoSegmentationRequest

import time
import numpy as np
# from apc_msgs.msg import DoObjectProposal


class DecideGraspPoseStateFromPointCloud(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'],
            input_keys=['object_2d_median','next_item_to_pick','cropped_cloud'],
            output_keys=['goal_pose', 'goal_pose_array', 'pre_grasp_pose_array', 'move_group_name_array']
        )

        # wait for the services to appear
        self.waitForService('/object_proposal_node/do_proposal')
        self.waitForService('/segmentation_node/do_segmentation')
        self.waitForService('/apc_grasping/detect_grasp_candidates')
        self.waitForService('/apc_grasping/grasp_selection_from_candidates')

        # create a proxy to that service
        self.srv_proposal = rospy.ServiceProxy('/object_proposal_node/do_proposal', apc_msgs.srv.DoObjectProposal)
        self.srv_segmentation = rospy.ServiceProxy('/segmentation_node/do_segmentation', apc_msgs.srv.DoSegmentation)
        self.srv_graspCandidates = rospy.ServiceProxy('/apc_grasping/detect_grasp_candidates', apc_msgs.srv.DetectGraspCandidates)
        self.srv_selectGraspFromCandidates = rospy.ServiceProxy('/apc_grasping/grasp_selection_from_candidates', apc_msgs.srv.SelectGraspFromCandidates)

        # setup subscribers for RGB image and point cloud topics
        self.sub_points = message_filters.Subscriber('/realsense/points_aligned', PointCloud2)
        self.sub_rgb = message_filters.Subscriber('/realsense/rgb/image_raw', Image)
        # self.sub_points = message_filters.Subscriber('/realsense/points_aligned_cropped', PointCloud2)
        # self.sub_rgb = message_filters.Subscriber('/realsense/rgb/image_raw_cropped', Image)

        self.points_pub = rospy.Publisher('pointsBeforeChris', PointCloud2, queue_size=10)

        # approximately synchronize RGB, and pointcloud topics
        self.ats = message_filters.ApproximateTimeSynchronizer([self.sub_rgb, self.sub_points], 1, 2.0)
        self.ats.registerCallback(self.RGBPoints_callback)


        self.ignore_rgbpoints = True
        self.current_rgb = None
        self.current_pointcloud = None


        self.tf_listener = TransformListener()

        self.maxTrys = 5

        rospy.loginfo('Ok.')

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

    # ==========================================================
    def RGBPoints_callback(self, rgb, pointcloud):
        if not self.ignore_rgbpoints:
            self.ignore_rgbpoints = True
            rospy.loginfo('Received topics.')
            self.current_rgb = rgb
            # self.current_pointcloud = pointcloud
        else:
            # rospy.loginfo('Ignoring incoming topics.')
            pass


    # ==========================================================
    def insideBoundingBox(self, p, bb):

        if not (p[0] >= bb.top_left.x and p[0] <= bb.bottom_right.x):
            return False
        else:
            if not (p[1] >= bb.top_left.y and p[1] <= bb.bottom_right.y):
                return False
            else:
                return True

    # ==========================================================
    def distFromCenter(self, p, bb):

        if not self.insideBoundingBox(p, bb):
            return np.inf
        else:
            center = np.array([(bb.bottom_right.x + bb.top_left.x) / 2.0, (bb.bottom_right.y + bb.top_left.y) / 2.0])
            pp = np.array(p)

            d = center - pp
            return np.sqrt(np.sum(d**2))

    # ==========================================================
    def execute(self, userdata):
        success = False
        try_counter = 0

        while not success and try_counter < self.maxTrys:
            self.current_rgb = None
            self.current_pointcloud = userdata['cropped_cloud']

            try_counter += 1

            rospy.loginfo("Looking for grasp poses. Try %d of %d." % (try_counter, self.maxTrys))

	        # get the current pointcloud and RGB image
            self.ignore_rgbpoints = False

            r = rospy.Rate(10)



            while self.current_rgb is None:
                rospy.loginfo('Waiting for synchronized RGB and Pointcloud topics ...')
                time.sleep(0.15)

            # call the DoSegmentation service, receive the segmented pointcloud in the response
            rospy.loginfo('Calling segmentation service ... ')
            req = DoSegmentationRequest()
            req.input_cloud = self.current_pointcloud
            segmented_cloud = self.srv_segmentation.call(req).segmented_cloud

            # form a PointcloudRGB message rofor the proposal service
            pointCloudRGB = PointcloudRGB(segmented_cloud, self.current_rgb)

            # call the DoProposal service, receive a ObjectProposals instance in the response
            rospy.loginfo('Calling object proposal service ... ')
            proposals = self.srv_proposal.call(pointCloudRGB).object_proposals

            # go through the bounding box coordinates, determine which our userdata['object_2d_median'] point is most central

            if 'object_2d_median' in userdata:
                p = userdata['object_2d_median']
            else:
                rospy.logwarn('No object median defined in the userdata!')
                p = [ 100, 100 ]      # Juxi's hack!

            distances = []
            for bb in proposals.bounding_boxes:
                distances.append(self.distFromCenter(p, bb))
            distances = np.array(distances)

            # get a handle to the corresponding point cloud segment
            if np.min(distances) == np.inf:
                rospy.logwarn("Object median point did not fall into any proposal box.")
                idx = None
            else:
                idx = np.argmin(distances)

            if idx is not None:
                pointcloud_segment = proposals.segments[idx]

                pointcloud_segment.header.frame_id = self.current_pointcloud.header.frame_id

                # call Chris' grasp pose service with that pointcloud
                rospy.loginfo('Calling grasp point detection service ... ')
                self.points_pub.publish(pointcloud_segment)
                grasp_candidates_response = self.srv_graspCandidates.call(pointcloud_segment, 0.01)

                if len(grasp_candidates_response.grasp_candidates.grasp_utilities) == 0:
                    rospy.logwarn("Could not find any suitable grasp pose!")
                    success = False
                    time.sleep(0.5)
                else:
                    bin_name = userdata['next_item_to_pick']['bin']
                    grasp_selection_response = self.srv_selectGraspFromCandidates.call(
                                grasp_candidates_response.grasp_candidates,
                                bin_name, {'left_arm','left_arm_90'})
                                # bin_name, {'left_arm'})

                    if grasp_selection_response.success.data == False:
                        print 'Grasp selection found no reachable grasps :(\n'
                        success = False
                        break

                    # print '\nGrasp Selection Response = '
                    # print grasp_selection_response
                    # print '---------------------\n'
                    # transform it into a fixed coordinate frame
                    # self.tf_listener.waitForTransform('/base', pose.header.frame_id, rospy.Time(0), rospy.Duration(4.0))
                    # common_time = self.tf_listener.getLatestCommonTime('/base',pose.header.frame_id)
                    # pose.header.stamp = common_time
                    # pose_fixed_frame = []
                    # for myPose in response.grasp_candidates.grasp_poses.poses:
                    #     pose.pose = myPose
                    #     # Need to do in z direction as we are in the camera frame
                    #     pose.pose.position.z += 0.04  # TODO replace with param
                    #     pose_fixed_frame.append(self.tf_listener.transformPose('/base', pose))

                    # store it for the other states in the state machine
                    userdata['goal_pose'] = grasp_selection_response.selected_grasps.poses[0]
                    userdata['goal_pose_array'] = grasp_selection_response.selected_grasps
                    userdata['pre_grasp_pose_array'] = grasp_selection_response.selected_pre_grasps
                    userdata['move_group_name_array'] = grasp_selection_response.grasp_move_group
                    # rospy.loginfo('Found best grasp pose at:')
                    # print pose_fixed_frame

                    success = True

        # clean up
        self.current_rgb = None
        self.current_pointcloud = None

        if success:
        	return 'succeeded'
        else:
        	return 'failed'


# ==========================================================
# ==========================================================
# ==========================================================
if __name__ == '__main__':

    rospy.init_node('grasp_decision_test')

    dg = DecideGraspPoseStateFromPointCloud()
    dg.execute(None)

    rospy.spin()
