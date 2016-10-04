import rospy
import smach
import smach_ros
import threading
import numpy as np
# import actionlib
# from apc_msgs.msg import FindObjects2DAction
# from apc_msgs.msg import FindObjects2DActionGoal

from apc_msgs.srv import DetectObject2D
from apc_msgs.srv import DetectObject2DRequest
from geometry_msgs.msg import Pose, Point, Quaternion

# ==========================================================
class DetectAllObjectsState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'no_objects', 'preempted', 'confidence_too_low'],
            input_keys=['next_item_to_pick'],
            output_keys=['object_2d_medians', 'object_2d_depths']
            )

        # self.ac = actionlib.SimpleActionClient('find_object_2d_wrapper', FindObjects2DAction)
        #
        # rospy.loginfo('Waiting for action server for find_object_2d_wrapper to come online ...')
        # if not self.ac.wait_for_server(timeout=rospy.Duration(2)):
        #     rospy.logerr('Action server for find_object_2d_wrapper not available. Restart and try again.')

    # ==========================================================
    def execute(self, userdata):
        detection_results_list = []

        for item_id in userdata['next_item_to_pick']['bin_contents_ids']:
            print 'Object ID to detect = ', item_id
            detection_results_list.append(self.detectObject2D(item_id))

        # process the results
        userdata['object_2d_depths'] = {}
        object_2d_depths = {}
        userdata['object_2d_medians'] = {}
        object_2d_medians = {}
        for detection_result in detection_results_list:
            if detection_result.success.data is True:
                print 'Object ID detected = ', detection_result.detectedObjectID
                print 'Object ID detection confidence = ', detection_result.confidenceOfObjectDetection

                confidenceThresholdDict = {
                    '8000': 30,
                    '9000': 30,
                    '13000': 30,
                    '24000': 30
                }

                if detection_result.confidenceOfObjectDetection.data < confidenceThresholdDict[str(detection_result.detectedObjectID.data)]:
                    print 'Object confidence too low\n'
                    return 'confidence_too_low'

                if np.isnan(detection_result.confidenceOfObjectDetection.data):
                    print 'Object confidence is nan\n'
                    return 'confidence_too_low'

                # userdata['object_2d_depths'][str(detection_result.detectedObjectID)] = detection_result.depthOfDetectedFeatureMedian.data
                object_2d_depths[str(detection_result.detectedObjectID.data)] = detection_result.depthOfDetectedFeatureMedian.data
                # userdata['object_2d_medians'][str(detection_result.detectedObjectID)] = [ detection_result.medianOfDetectedFeatures[0].data, detection_result.medianOfDetectedFeatures[1].data ]
                object_2d_medians[str(detection_result.detectedObjectID.data)] = [ detection_result.medianOfDetectedFeatures[0].data, detection_result.medianOfDetectedFeatures[1].data ]
            else:
                rospy.logwarn('Not all 2D object models found.')
                userdata['object_2d_medians'] = {}
                return 'no_objects'

        userdata['object_2d_depths'] = object_2d_depths
        userdata['object_2d_medians'] = object_2d_medians
        return 'succeeded'

    # ==========================================================
    def detectObject2D(self, object_id_to_pick):
        rospy.wait_for_service('detect_object_2d')
        try:
            detect_object_2d_service = rospy.ServiceProxy('detect_object_2d', DetectObject2D)
            request = DetectObject2DRequest() # Get request message object
            object_id_to_pick = object_id_to_pick
            request.pleaseDetectObjectID.data = object_id_to_pick
            print 'object id to pick = ', object_id_to_pick
            for i in range(10):  # HACK for allowing find_object_2d to load the new database
                resp = detect_object_2d_service(request)
            return resp
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
