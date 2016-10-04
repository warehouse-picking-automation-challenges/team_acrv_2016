import rospy
import smach

from apc_msgs.msg import Digital
import time


class CheckPressureSensorState(smach.State):
    def __init__(self, state=None, movegroup=None):
        smach.State.__init__(self, outcomes=['succeeded','failed'])

        self.current_state = False
        self.pumpPublisher = rospy.Subscriber('/arduino_node/sensor/Pressure_Sensor',
                                Digital, self.sensor_callback)


    # ==========================================================

    def sensor_callback(self, msg):
        self.current_state = msg.value != 0

    def execute(self, userdata):
        time.sleep(2) # temp hack
        if self.current_state:
            print "---------------------\nPRESSURE SENSOR ON\n-------------------\n"
            return 'succeeded'
        else:
            print "---------------------\nPRESSURE SENSOR OFF\n-------------------\n"
            return 'failed'
