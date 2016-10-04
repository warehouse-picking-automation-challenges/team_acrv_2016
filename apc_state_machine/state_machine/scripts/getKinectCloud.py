import rospy
import smach
from sensor_msgs.msg import PointCloud2

class GetKinectCloud(smach.State):
    def __init__(self, action='get_kinect_cloud'):
        smach.State.__init__(self, input_keys=['next_item_to_pick'],
                output_keys=['cloud_data'], outcomes=['succeeded', 'failed'])

        self.action_ = action

        if self.action_ == 'get_kinect_cloud':
            self.kinect_sub = rospy.Subscriber('/kinect2/qhd/points',PointCloud2,self.points_callback)

        self.cloud = False
        self.count = 0


    def points_callback(self, msg):
        if self.count < 5:
            self.count = self.count + 1
            return
        self.points = msg
        self.cloud = True

    # ==========================================================
    def execute(self, userdata):

        try:
            while not self.cloud:
                donothing = 0
            userdata['cloud_data'] = {'full_cloud':self.points}
            print "Kinect cloud height: ", self.points.height, "\nKinect cloud width: ", self.points.width
            self.count = 0
            return 'succeeded'
        except:
            print "Get Kinfu Cloud state failed!!!"
            return 'failed'
