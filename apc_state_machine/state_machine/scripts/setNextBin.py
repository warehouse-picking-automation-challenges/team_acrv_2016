
import smach

class SetNextBin(smach.State):
    def __init__(self, action='nothing'):
        smach.State.__init__(self, output_keys=['next_item_to_pick'],
                             outcomes=['succeeded', 'failed'])
        self.current_bin = 2
        # self.current_bin = 8
        self.bin_names = ['A','B','C','D','E','F','G','H','I','J','K','L']
        # self.bin_names = ['A','B','D','E','F','G','H','I','J','K','L']
        # self.bin_names = ['D','E','F','G','H','I','J','K','L']


    # ==========================================================
    def execute(self, userdata):
        if self.current_bin == 11:
        # if self.current_bin == 8:
            self.current_bin = 0
            # wait = raw_input('Press <ENTER> to continue')
        else:
            self.current_bin = self.current_bin + 1
        userdata['next_item_to_pick'] = {'bin':'bin_' + self.bin_names[self.current_bin]}
        return 'succeeded'
