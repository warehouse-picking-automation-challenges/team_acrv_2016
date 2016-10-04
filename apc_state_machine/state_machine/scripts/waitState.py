#from spaceBot import *
import rospy
import smach
import smach_ros
import threading


class WaitState(smach.State):  
  def __init__(self, duration=1.0):    
    smach.State.__init__(self, outcomes=['succeeded', 'preempted'], input_keys=['message'])
  
    self.duration = duration  
    self.trigger = threading.Event()

  # ========================================================== 
  def execute(self, userdata):

    # wait for the specified duration or until we are asked to preempt
    self.trigger.wait(self.duration)
      
    if self.preempt_requested():
      self.service_preempt()
      return 'preempted'
    else:
      return 'succeeded'

  
  # ========================================================== 
  def request_preempt(self):
    smach.State.request_preempt(self) 
    # rospy.logwarn('request preempt')
    self.trigger.set()

    
