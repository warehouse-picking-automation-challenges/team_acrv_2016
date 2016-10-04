import rospy
import smach


class PublisherState(smach.State):
    def __init__(self, topic, datatype, data, n=1, hz=1):    
        smach.State.__init__(self, outcomes=['succeeded', 'aborted']
        )
        
        self.data = data
        self.n = n          # how often? publish n times
        self.hz = hz    # 
        self.datatype = datatype
        self.publisher = rospy.Publisher(topic, datatype, queue_size=10)

    # ========================================================== 
    def execute(self, userdata):

        r = rospy.Rate(self.hz)
        for i in range(self.n):
            self.publisher.publish(self.datatype(**self.data))
            r.sleep()

        return 'succeeded'


