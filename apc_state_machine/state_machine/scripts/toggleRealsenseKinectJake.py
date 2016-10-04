import rospy
import smach

from std_srvs.srv import SetBool, SetBoolRequest
from std_srvs.srv import Empty as EmptySrv
from std_srvs.srv import EmptyRequest
from std_msgs.msg import Empty
from std_msgs.msg import String
from apc_msgs.srv import Camera_Info,Camera_InfoRequest
from sensor_msgs.msg import CameraInfo

class ToggleRealsenseKinectJake(smach.State):
    def __init__(self, action='realsense_off_kinect_on'):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])

        # wait for the service to appear
        rospy.loginfo('Waiting for fill/unfill bins and toggle tote to come up ...')

        self.action_ = action
        realsense_srv_name = '/realsense/enable_disable'
        kinect_topic = '/kinect_toggle'
        kinfu_topic = '/ros_kinfu/reset'
        kinfu_change_camera_srv_name = '/ros_kinfu/change_camera'
        objprop_change_camera_srv_name = '/object_proposal_node/change_camera_info_topic'

        if self.action_ == 'realsense_off_kinect_on':
            camera_info_topic = '/kinect2/qhd/camera_info'
        if self.action_ == 'realsense_on_kinect_off':
            camera_info_topic = '/realsense/rgb/camera_info'

        self.camera_info_received = False
        self.camera_info_msg = Camera_InfoRequest()

        try:
            rospy.wait_for_service(realsense_srv_name, timeout=1)
            rospy.wait_for_service(kinfu_change_camera_srv_name, timeout=1)
            rospy.wait_for_service(objprop_change_camera_srv_name, timeout=1)

        except:
            rospy.logerr('Service of %s not available. Restart and try again.' % realsense_srv_name)

        self.realsense_service = rospy.ServiceProxy(realsense_srv_name, SetBool)
        self.kinect_pub = rospy.Publisher(kinect_topic, String)
        self.kinfu_change_camera_srv = rospy.ServiceProxy(kinfu_change_camera_srv_name,EmptySrv)
        self.objprop_change_camera_srv = rospy.ServiceProxy(objprop_change_camera_srv_name,Camera_Info)
        self.kinfu_reset_pub = rospy.Publisher(kinfu_topic,Empty)
        self.camera_info_sub = rospy.Subscriber(camera_info_topic,CameraInfo,self.camera_info_callback)

    def camera_info_callback(self,msg):

        if not self.camera_info_received:
            print "CameraInfo received"
            self.camera_info_msg.camera_info = msg
            self.camera_info_received = True

    # ==========================================================
    def execute(self, userdata):
        arbitrary = input("pressToContinue")
        realsense_request = SetBoolRequest()
        kinect_request = String()
        kinfu_reset_request = Empty()
        kinfu_change_camera_request = EmptyRequest()

        if self.action_ == 'realsense_off_kinect_on':
            realsense_request.data = False
            kinect_request.data = 'on'
            rospy.set_param('/ros_kinfu/depth_image_topic','/kinect2/qhd/image_depth_rect')
            rospy.set_param('/ros_kinfu/rgb_image_topic','/kinect2/qhd/image_color')
            rospy.set_param('/ros_kinfu/camera_info_topic','/kinect2/qhd/camera_info')
            rospy.set_param('/ros_kinfu/camera_frame_id','/kinect2_rgb_optical_frame')
            rospy.set_param('/ros_kinfu/volume_size',1.0)


        elif self.action_ == 'realsense_on_kinect_off':
            realsense_request.data = True
            kinect_request.data = 'off'
            rospy.set_param('/ros_kinfu/depth_image_topic','/realsense/depth/image_raw')
            rospy.set_param('/ros_kinfu/rgb_image_topic','/realsense/rgb_depth_aligned/image_raw')
            rospy.set_param('/ros_kinfu/camera_info_topic','/realsense/depth/camera_info')
            rospy.set_param('/ros_kinfu/camera_frame_id','/camera_depth_optical_frame')
            rospy.set_param('/ros_kinfu/volume_size',0.85)

        else:
            return 'failed'

        try:
            response_realsense = self.realsense_service.call(realsense_request)
        except:
            return 'failed'

        self.kinect_pub.publish(kinect_request)
        while not self.camera_info_received:
            print "Waiting for CameraInfo"
        print self.camera_info_msg
        self.objprop_change_camera_srv.call(self.camera_info_msg)
        self.kinfu_reset_pub.publish(kinfu_reset_request)
        # self.kinfu_change_camera_srv(kinfu_change_camera_request)
        self.camera_info_received = False

        if response_realsense.success:
            return 'succeeded'
        else:
            return 'failed'
