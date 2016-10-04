import rospy
import smach
import smach_ros

from apc_msgs.msg import PointcloudRGB
from apc_msgs.srv import DoObjectProposal,DoObjectProposalRequest

import time
import numpy as np
from sklearn.cluster import KMeans
# from apc_msgs.msg import DoObjectProposal
from matplotlib import pyplot as plt
from cv_bridge import CvBridge
import cv2


class ObjectProposals(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'],
            input_keys=['cloud_data'],
            output_keys=['object_proposals','most_central']
        )

        # wait for the services to appear
        self.waitForService('/object_proposal_node/do_proposal')
        # create a proxy to that service
        self.srv_proposal = rospy.ServiceProxy('/object_proposal_node/do_proposal', DoObjectProposal)

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
    def execute(self, userdata):

        try:
            # form a PointcloudRGB message rofor the proposal service
            pointCloudRGB = PointcloudRGB()
            request = DoObjectProposalRequest()
            pointCloudRGB.pointcloud = userdata['cloud_data']['segmented_cloud']
            pointCloudRGB.image = userdata['cloud_data']['image']
            request.pointcloud_rgb = pointCloudRGB

            response = self.srv_proposal.call(request)
            userdata['object_proposals'] = response.object_proposals
            userdata['most_central'] = response.most_central

            return 'succeeded'
        except:
            print "Object proposal state failed!!!"
            return 'failed'
