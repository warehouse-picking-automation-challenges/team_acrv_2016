import rospy
import smach


class PreFillUserDataState(smach.State):
    def __init__(self, dict_of_userdata_keys):
        smach.State.__init__(self,
                             outcomes=['succeeded','failed'],
                             output_keys=['map_key_0',
                                          'map_key_1',
                                          'map_key_2',
                                          'map_key_3',
                                          'map_key_4',
                                          'map_key_5',
                                          'map_key_6'])

        self.dict_of_userdata_keys = dict_of_userdata_keys
        self.output_key_prefix = 'map_key_'
        self.num_output_userdata_keys = 7

    # ==========================================================
    def execute(self, userdata):
        if len(self.dict_of_userdata_keys) > self.num_output_userdata_keys:
            rospy.logerr('You need to create more output keys in the PreFillUserDataState!!!')
            return 'failed'
        i = 0
        for key, value in self.dict_of_userdata_keys.iteritems():
            map_key_name = self.output_key_prefix + str(i)
            userdata[map_key_name] = value
            i = i + 1
        return 'succeeded'
