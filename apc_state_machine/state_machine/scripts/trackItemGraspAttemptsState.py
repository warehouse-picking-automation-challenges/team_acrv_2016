import smach

# ==========================================================
class TrackItemGraspAttemptsState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['try_this_item_again','move_on_to_next_item'],
            input_keys=['current_item_attempts'],
            output_keys=['current_item_attempts']
            )

    # ==========================================================
    def execute(self, userdata):
        if not userdata['current_item_attempts']:
            self.current_item_attempts = 0

        # self.lastItemAttempted = userdata['next_item_to_pick']['item']
        self.current_item_attempts = self.current_item_attempts + 1
        userdata['current_item_attempts'] = self.current_item_attempts

        # Note, grasp selection will move to object centroid on the third failed attempt
        if self.current_item_attempts >= 3:
            self.current_item_attempts = 0
            return 'move_on_to_next_item'
        else:
            return 'try_this_item_again'
