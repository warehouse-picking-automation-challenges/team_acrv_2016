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
class DetectObjectState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'no_objects', 'preempted', 'confidence_too_low'],
            input_keys=['next_item_to_pick'],
            output_keys=['goal_frame_id','goal_pose','goal_confidence','object_2d_median']
            )

        # self.ac = actionlib.SimpleActionClient('find_object_2d_wrapper', FindObjects2DAction)
        #
        # rospy.loginfo('Waiting for action server for find_object_2d_wrapper to come online ...')
        # if not self.ac.wait_for_server(timeout=rospy.Duration(2)):
        #     rospy.logerr('Action server for find_object_2d_wrapper not available. Restart and try again.')

    # ==========================================================
    def execute(self, userdata):
        object_id_to_pick = userdata['next_item_to_pick']['id']
        print 'Object ID to pick = ', object_id_to_pick
        detection_result = self.detectObject2D(object_id_to_pick)

        # process the results
        if detection_result.success.data is True:
            print 'Object ID detected = ', detection_result.detectedObjectID
            print 'Object ID detection confidence = ', detection_result.confidenceOfObjectDetection
            confidenceThresholdDict = {
                '8000': 30,
                '9000': 30,
                '13000': 50,
                '24000': 30,
                '37000': 30
            }
            if detection_result.confidenceOfObjectDetection.data < confidenceThresholdDict[str(detection_result.detectedObjectID.data)]:
                print 'Object confidence too low\n'
                return 'confidence_too_low'
            if np.isnan(detection_result.confidenceOfObjectDetection.data):
                print 'Object confidence is nan\n'
                return 'confidence_too_low'

            print 'Object feature median X: %s Y: %s' % (detection_result.medianOfDetectedFeatures[0], detection_result.medianOfDetectedFeatures[1])
            # userdata['detected_2D_object_info'] = [detection_result.detectedObjectID, detection_result.confidenceOfObjectDetection]
            userdata['goal_frame_id'] = 'object_' + str(detection_result.detectedObjectID.data)
            # TODO replace quaternion below with proper tf in find_object_2d
            userdata['goal_pose'] = Pose(position=Point(0,0,0), orientation=Quaternion(0,0,1,0))
            userdata['goal_confidence'] = detection_result.confidenceOfObjectDetection.data
            userdata['object_2d_median'] = [ detection_result.medianOfDetectedFeatures[0].data, detection_result.medianOfDetectedFeatures[1].data ]
            return 'succeeded'
        else:
            rospy.logwarn('No 2D object models found.')
            # userdata['detected_2D_object_info'] = []
            userdata['goal_frame_id'] = ''
            userdata['goal_pose'] = ''
            userdata['goal_confidence'] = 0.0
            return 'no_objects'

    # ==========================================================
    def detectObject2D(self, object_id_to_pick):
        rospy.wait_for_service('detect_object_2d')
        try:
            detect_object_2d_service = rospy.ServiceProxy('detect_object_2d', DetectObject2D)
            request = DetectObject2DRequest() # Get request message object
            object_id_to_pick = object_id_to_pick
            request.pleaseDetectObjectID.data = object_id_to_pick
            print 'object id to pick = ', object_id_to_pick
            resp = detect_object_2d_service(request)
            return resp
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
