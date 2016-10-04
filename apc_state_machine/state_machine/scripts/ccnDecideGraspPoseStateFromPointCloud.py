import rospy
import smach
import smach_ros

import message_filters

import tf
from tf import transformations
from tf import TransformListener
from tf import transformations

from geometry_msgs.msg import PoseStamped

from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import Image
from apc_msgs.msg import PointcloudRGB
from apc_msgs.msg import ObjectProposals
from apc_msgs.msg import GraspCandidates

import time
from apc_msgs.srv import ClassifyRegions, ClassifyRegionsRequest, ClassifyRegionsResponse, DetectGraspCandidates, SelectGraspFromCandidates, DoObjectProposal, DoSegmentation


class CNNDecideGraspPoseStateFromPointCloud(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'],
            input_keys=['next_item_to_pick'],
            output_keys=['goal_pose', 'goal_pose_array','pre_grasp_pose_array', 'move_group_name_array'])

        # get object ids from param server
        self.object_ids = rospy.get_param('object_ids')

        # wait for the services to appear
        self.waitForService('/classifier_node/classify_regions')
        self.waitForService('/object_proposal_node/do_proposal')
        self.waitForService('/segmentation_node/do_segmentation')
        self.waitForService('/apc_grasping/detect_grasp_candidates')
        self.waitForService('/apc_grasping/grasp_selection_from_candidates')

        # create a proxy to that service
        self.cnn_service=rospy.ServiceProxy('/classifier_node/classify_regions', ClassifyRegions)
        self.srv_proposal=rospy.ServiceProxy('/object_proposal_node/do_proposal', DoObjectProposal)
        self.srv_segmentation=rospy.ServiceProxy('/segmentation_node/do_segmentation', DoSegmentation)
        self.srv_graspCandidates=rospy.ServiceProxy('/apc_grasping/detect_grasp_candidates', DetectGraspCandidates)
        self.srv_selectGraspFromCandidates=rospy.ServiceProxy('/apc_grasping/grasp_selection_from_candidates', SelectGraspFromCandidates)

        # setup subscribers for RGB image and point cloud topics
        self.sub_points=message_filters.Subscriber('/realsense/points_aligned', PointCloud2)
        self.sub_rgb=message_filters.Subscriber('/realsense/rgb/image_raw', Image)

        # approximately synchronize RGB, and pointcloud topics
        self.ats=message_filters.ApproximateTimeSynchronizer([self.sub_rgb, self.sub_points], 1, 0.5)
        self.ats.registerCallback(self.RGBPoints_callback)


        self.ignore_rgbpoints = True
        self.current_rgb = None
        self.current_pointcloud = None
        self.maxTrys = 5

    # ==========================================================
    def waitForService(self, service):

        rospy.loginfo('Waiting for service %s to come online ...' % service)
        try:
            rospy.wait_for_service(service, timeout=0.1)
        except:
            rospy.logerr(
                'Service %s not available. Restart and try again.' % service)
            return False
        else:
            return True

    # ==========================================================
    def RGBPoints_callback(self, rgb, pointcloud):
        if not self.ignore_rgbpoints:
            self.ignore_rgbpoints=True
            rospy.loginfo('Received topics.')
            self.current_rgb=rgb
            self.current_pointcloud=pointcloud
        else:
            # rospy.loginfo('Ignoring incoming topics.')
            pass

    # ==========================================================
    def execute(self, userdata):

    	success=False
    	try_counter=0

        object_id_to_pick = userdata['next_item_to_pick']['id']
        object_string = self.object_ids.keys()[self.object_ids.values().index(object_id_to_pick)]

        while not success and try_counter < self.maxTrys:
            self.current_rgb=None
            self.current_pointcloud=None

            try_counter += 1

            rospy.loginfo("Looking for grasp poses. Try %d of %d." %
                          (try_counter, self.maxTrys))

	        # get the current pointcloud and RGB image
            self.ignore_rgbpoints=False

            r=rospy.Rate(10)
            while self.current_rgb is None or self.current_pointcloud is None:
                rospy.loginfo(
                    'Waiting for synchronized RGB and Pointcloud topics ...')
                time.sleep(0.25)

            # call the DoSegmentation service, receive the segmented pointcloud
            # in the response
            rospy.loginfo('Calling segmentation service ... ')
            segmented_cloud=self.srv_segmentation.call(
                self.current_pointcloud).segmented_cloud

            # form a PointcloudRGB message rofor the proposal service
            pointCloudRGB=PointcloudRGB(segmented_cloud, self.current_rgb)

            # call the DoProposal service, receive a ObjectProposals instance
            # in the response
            rospy.loginfo('Calling object proposal service ... ')
            proposals=self.srv_proposal.call(pointCloudRGB).object_proposals

            cnn_request = ClassifyRegionsRequest()
            cnn_request.object_id.data = object_string
            cnn_request.image = self.current_rgb
            cnn_request.proposals = proposals

            cnn_response = self.cnn_service.call(cnn_request)

            pointcloud_segment=cnn_request.segment

            # call Chris' grasp pose service with that pointcloud
            rospy.loginfo('Calling grasp point detection service ... ')
            self.points_pub.publish(pointcloud_segment)
            grasp_candidates_response=self.srv_graspCandidates.call(
                pointcloud_segment, 0.01)

            if len(grasp_candidates_response.grasp_candidates.grasp_utilities) == 0:
                rospy.logwarn("Could not find any suitable grasp pose!")
                success=False
                time.sleep(0.5)
            else:
                grasp_selection_response=self.srv_selectGraspFromCandidates.call(
                    grasp_candidates_response.grasp_candidates)

                if grasp_selection_response.success.data == False:
                    print 'Grasp selection found no reachable grasps :(\n'
                    success=False
                    break

                print '\nGrasp Selection Response = '
                print grasp_selection_response
                print '---------------------\n'

                # store it for the other states in the state machine
                userdata['goal_pose']=grasp_selection_response.selected_grasps.poses[0]
                userdata['goal_pose_array']=grasp_selection_response.selected_grasps
                userdata['pre_grasp_pose_array']=grasp_selection_response.selected_pre_grasps
                userdata['move_group_name_array']=grasp_selection_response.grasp_move_group

                success=True

        # clean up
        self.current_rgb=None
        self.current_pointcloud=None

        if success:
        	return 'succeeded'
        else:
        	return 'failed'


# ==========================================================
# ==========================================================
# ==========================================================
if __name__ == '__main__':

    rospy.init_node('cnn_grasp_decision_test')

    dg=CNNDecideGraspPoseStateFromPointCloud()
    dg.execute(None)

    rospy.spin()
