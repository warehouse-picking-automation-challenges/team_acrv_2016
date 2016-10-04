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


class SegmentPointcloud(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'],
            input_keys=['cloud_data'],
            output_keys=['cloud_data']
        )

        # wait for the services to appear
        self.waitForService('/segmentation_node/do_segmentation')

        # create a proxy to that service
        self.srv_segmentation = rospy.ServiceProxy('/segmentation_node/do_segmentation', DoSegmentation)

        self.segmented_cloud_count = 10

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
    def execute(self, userdata):

        try:
            req = DoSegmentationRequest()
            req.input_cloud = userdata['cloud_data']['cropped_cloud']
            response = self.srv_segmentation.call(req)
            userdata['cloud_data']['segmented_cloud'] = response.segmented_cloud
            success = True
            return 'succeeded'
        except:
            return 'failed'
