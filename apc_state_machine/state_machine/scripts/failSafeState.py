import rospy
import smach
import threading

class FailSafeState():
    """ Use parameter escalate_to in constructor to specify which outcome escalates to another outcome after n repetitions:
        For example, escalate_to={'failed':['abort', 5]}  returns 'failed' max 5 times in a row, but will return 'abort' the 6th time.

        Usage:
        - Derive your custom states from this class AND from smach.State, e.g. class TestState(smach.State, FailSafeState)
        - In your constructor, call FailSafeState.__init__(self, escalate_to=..., reset_with=...)
        - In the derived states, instead of doing return 'failed', call return self.outcome('failed')
        - See below (in the __main__ part) for an example


        Reset behaviour:
        - returning 4x 'failed', and then 'succeeded' will reset the counter for 'failed' to 0
        - returning 5x 'failed', and then escalating to 'abort' the 6th time will normally reset the counter for 'failed'

        - both points above are not true when 'failed' is a key in .reset_with
          E.g. with reset_with={'failed':['succeeded']}, the counter for 'failed' is only reset when 'succeeded' is returned
    """
    def __init__(self, escalate_to={}, reset_with={}):
       
        # count how often an outcome occurred in a row
        self.outcome_counter = {}

        # count how often an outcome occurred in total (this never gets reset)
        self.total_outcome_counter = {}
      
        self.escalate_to = escalate_to            
        self.reset_with = reset_with

        self.check_outcomes()

    # =========================================================================
    def check_outcomes(self):
        """ Check that all outcomes have been correctly registered to avoid crashes during runtime """
        if not hasattr(self, '_outcomes'):
            rospy.logwarn('Constructor of FailSafeState was called before the constructor of smach.State. This is dangerous, checking of registered outcomes is disabled and might lead to runtime crashes.')
        else:
            for escalated_outcome in [x[0] for x in  self.escalate_to.values()]:
                if escalated_outcome not in self.get_registered_outcomes():
                    rospy.logerr('Escalated outcome %s was not registered in this instance of FailSafeState! The state machine will crash when this outcome is encountered. Update your code!' % escalated_outcome)

    # =========================================================================
    def outcome(self, outcome, update_total_counter=True):
        """ Call this function instead of return in the derived child states.
            E.g. instead of doing return 'failed', call return self.outcome('failed')
        """

        self.check_outcomes()

        # if we never encountered this outcome before, initialise a new counter for it
        if outcome not in self.outcome_counter:
            self.outcome_counter[outcome] = 0

        # increment the counter for the requested outcome
        self.outcome_counter[outcome] += 1

        # check if we have to escalate to a different outcome
        if outcome in self.escalate_to:
            if self.outcome_counter[outcome] > self.escalate_to[outcome][1]:
                outcome = self.outcome(self.escalate_to[outcome][0], update_total_counter=False)

                # make sure we have a counter variable for the new outcome, init it to 1
                if outcome not in self.outcome_counter:
                    self.outcome_counter[outcome] = 1

        # reset the counter for all other outcomes:
        for k in self.outcome_counter:
            if k != outcome:
                if k in self.reset_with:
                    if outcome in self.reset_with[k]:
                      self.outcome_counter[k] = 0
                else:
                    self.outcome_counter[k] = 0

        # keep track of how often we returned a certain outcome in total (gets never reset)
        if update_total_counter:
            self.total_outcome_counter[outcome] = self.total_outcome_counter.get(outcome, 0) + 1

        # and we are done
        return outcome

# =========================================================================
# =========================================================================
# =========================================================================
if __name__ == '__main__':

    # === A demo state to show how to use the FailSafeState within SMACH
    class TestState(smach.State, FailSafeState):
        def __init__(self, escalate_to={}, reset_with={}):

            smach.State.__init__(self, outcomes=['succeeded', 'aborted', 'failed'])
            FailSafeState.__init__(self, escalate_to=escalate_to, reset_with=reset_with)

        # =========================================================================
        def execute(self, userdata):
            return self.outcome('failed')

    # =========================================================================
        
    # === test program

    rospy.init_node('smach_failsafe_test')

    # Create the top level SMACH state machine
    sm = smach.StateMachine(outcomes=['succeeded', 'aborted', 'preempted'])

    # create states and connect them
    with sm:

        sm.add('fail_twice', TestState(escalate_to={'failed':['aborted', 2]}), transitions={'failed': 'fail_twice', 'aborted':'repeat_three_times'})

        sm.add('repeat_three_times', TestState(escalate_to={'failed':['aborted', 3]}), transitions={'failed': 'fail_twice'})

    # run the state machine
    #   We start it in a separate thread so we can cleanly shut it down via CTRL-C
    #   by requesting preemption.
    #   The state machine normally runs until a terminal state is reached.
    smach_thread = threading.Thread(target=sm.execute)
    smach_thread.start()

    # Wait for ctrl-c to stop the application
    rospy.spin()

    # request the state machine to preempt and wait for the SM thread to finish
    sm.request_preempt()
    smach_thread.join()


    # fs = FailSafeState(escalate_to={'failed':['abort',2], 'abort':['self_destruct',2]}, reset_with={'failed':['succeeded']})
    # print fs.outcome('failed')
    # print fs.outcome('failed')
    # print fs.outcome('failed')
    # print fs.outcome('failed')
    # print fs.outcome('succeeded')
    # print fs.outcome('failed')
    # print fs.outcome('failed')
    # print fs.outcome('failed')
    # print fs.outcome('failed')
    # print fs.outcome('failed')
    # print fs.outcome('failed')
    # print fs.outcome('failed')
    # print fs.outcome('failed')

    # print fs.outcome_counter
    # print fs.total_outcome_counter

    # print fs.escalate_to

