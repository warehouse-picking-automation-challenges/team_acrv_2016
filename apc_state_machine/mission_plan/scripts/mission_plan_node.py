#!/usr/bin/env python

import rospy

import json
from mission_plan.srv import *
import operator

_items = [
    # PERFECT
    'kleenex_tissue_box',
    'expo_dry_erase_board_eraser',
    'soft_white_lightbulb',

    # GREAT
    'dove_beauty_bar',
    'folgers_classic_roast_coffee',
    'up_glucose_bottle',
    'elmers_washable_no_run_school_glue',
    'staples_index_cards',
    'kleenex_paper_towels',
    'peva_shower_curtain_liner',
    'crayola_24_ct',
    'jane_eyre_dvd',
    'laugh_out_loud_joke_book',
    'i_am_a_bunny_book',

    # GOOD
    'ticonderoga_12_pencils',
    'rawlings_baseball',
    'command_hooks',
    'scotch_bubble_mailer',

    # OKAY
    'creativity_chenille_stems',
    'safety_first_outlet_plugs',

    # AVERAGE
    'hanes_tube_socks',
    'clorox_utility_brush',
    'dr_browns_bottle_brush',

    # MILD
    'woods_extension_cord',
    'cool_shot_glue_sticks',
    'easter_turtle_sippy_cup',
    'scotch_duct_tape',
    'oral_b_toothbrush_green',
    'oral_b_toothbrush_red',
    'rolodex_jumbo_pencil_cup',

    # HARD
    'fiskars_scissors_red',
    'cherokee_easy_tee_shirt',
    'barkely_hide_bones',
    'platinum_pets_dog_bowl',
    'dasani_water_bottle',
    'kyjen_squeakin_eggs_plush_puppies',

    # WHOA
    'cloud_b_plush_bear',
    'womens_knit_gloves',
    'fitness_gear_3lb_dumbbell']


unpickable = [
    'fitness_gear_3lb_dumbbell',
    'womens_knit_gloves',
    'cloud_b_plush_bear',
    'fiskars_scissors_red',
    'kyjen_squeakin_eggs_plush_puppies',
    'barkely_hide_bones',
    'rolodex_jumbo_pencil_cup']

class MissionPlan:
    def __init__(self):
        # read the mission file name from the ROS parameter server
        try:
            mission_filename = rospy.get_param('~mission_file')
        except:
            rospy.logerr('Required parameter mission_file not found.')
            mission_filename = None

        # get more parameters
        self.mission_strategy = rospy.get_param('~mission_strategy', 'default')
        # task is decided by the json file\:
        # is there a work_order in the file? --> picking
        # is there tote_contents in the file? --> stowing
        # self.task = rospy.get_param('~task', 'pick_task')

        self.max_trials = rospy.get_param('~max_trials', 3)

        # keep track of how often we try to pick one particular object
        self.trials = {}

        # read the mission description from the provided json file
        self.mission_description = self.readJSON(mission_filename)


        # keep track of the bin and tote contents
        self.bin_contents = self.mission_description['bin_contents']

        # Publish to param server as can't send python dicts in ros msgs by default
        rospy.set_param('/mission_plan/bin_contents', self.bin_contents)

        # if not 'tote' in self.bin_contents:
        #     self.bin_contents['tote'] = []
        if 'tote_contents' in self.mission_description and len(self.mission_description['tote_contents']) != 0:
            self.bin_contents['tote'] = self.mission_description['tote_contents']
        else:
            self.modifyWorkOrder(self.mission_description)
            self.bin_contents['tote'] = []

        # print 'bin contents the first = ', self.bin_contents

        # decide on a mission plan (order in which to pick / store the items)
        self.mission_plan = self.createMissionPlan(self.mission_description)

        # register our services
        self.srv_get_bin_contents = rospy.Service('mission_plan/get_bin_contents', GetBinContents, self.getBinContents)
        self.srv_get_item_id_from_name = rospy.Service('mission_plan/get_item_id_from_name', GetItemIDFromName, self.getItemIDFromName)
        self.srv_get = rospy.Service('mission_plan/get_next_item', GetNextItem, self.getNextItem)
        self.srv_pop = rospy.Service('mission_plan/pop_item', PopItem, self.popItem)
        self.srv_move = rospy.Service('mission_plan/move_item', MoveItem, self.moveItem)
        self.srv_get_tote_items = rospy.Service('mission_plan/get_tote_items', GetToteItems, self.getToteItems)

        # TODO add also to write a new json file here!

        rospy.loginfo('mission_plan ready')

    # =========================================================================
    def readJSON(self, filename):
        try:
            return json.load(file(filename, 'r'))
        except:
            rospy.logerr('Fatal Error: Could not read mission description from file %s' % filename)
            return None

    # =========================================================================
    def modifyWorkOrder(self,data):
        # data['work_order'][0]['item'] = 'TheBiggestSandwhich'
        # for order in data['work_order']:
            # order['item']
            # check where in order the item on the list is
            # put to the top of new list
            # check for doubles / none

        # add difficulty field to work order
        # difficulty based off shelf fullness and item difficulty (rank)
        # shelf fullness based off shelf item volumes

        # if there are 10 items on the shelf, @max pick item 20
        # thus each item on shelf = + 2 to rank
        # if item1 has one other item on shelf, item2 does not
        # rank1 = 2, rank2 = 1, rank 3 = 2
        item_count_weight = 2 # every item lets it go down the ranks 2 more
        item_rank_weight = 1

        new_order = []

        for order in data['work_order']:
            if order['item'] in unpickable:
                continue
            order['rank'] = _items.index(order['item']) * item_rank_weight
            order['rank'] += len(data['bin_contents'][order['bin']]) * item_rank_weight
            new_order.append(order)

        # for index, item in enumerate(_items):
        #     for order in data['work_order']:
        #         if order['item'] == item:
        #             order['rank'] = index;
        #             new_order.append(order)

        new_order.sort(key=operator.itemgetter('rank'))

        data['work_order'] = new_order
        print new_order

    # =========================================================================
    def createMissionPlan(self, mission_description):
        if mission_description is None:
            rospy.logerr('Invalid mission description.')
            return

        # set task depending on the json file, maybe force from the outside?
        if 'work_order' in self.mission_description:
            self.task = 'pick_task'
        else:
            self.task = 'stow_task'

        print "Task: ", self.task

        if self.task == 'stow_task':
            # todo maybe add some strategy here?
            return self.mission_description['tote_contents']
        else:
            if self.mission_strategy == 'conservative':
                rospy.loginfo('Creating a mission plan, following the "%s" strategy...' % self.mission_strategy)
                # implement a more fancy strategy here, e.g. pick the easy objects first or whatever
                # use the ~mission_strategy parameter to switch between different strategies
                return []
            elif self.mission_strategy == 'easy_first':
                rospy.loginfo('Creating a mission plan, following the "%s" strategy...' % self.mission_strategy)
                return []
            else:
                rospy.loginfo('Creating a mission plan, following the default strategy...')
                # this is the default behaviour, just pick objects in the order they are given in the json file
                return self.mission_description['work_order']


    # =========================================================================
    def getToteItems(self, request):

        response = GetToteItemsResponse()
        response.tote_contents = self.mission_description['tote_contents']
        print 'bin contents the second = ', self.bin_contents
        response.bin_contents = self.bin_contents

        return response


    # =========================================================================
    def moveItem(self, request):

        response = MoveItemResponse()

        # check that the requested item is in the bin we want to move it from
        if (request.from_bin not in self.bin_contents) or (request.to_bin not in self.bin_contents):
            rospy.logerr('Unknown bin (%s or %s) referenced in MoveItem service!' % (request.from_bin, request.to_bin))
            response.success = False
        else:
            if request.item not in self.bin_contents[request.from_bin]:
                rospy.logerr('Item %s requested to be moved from %s, but it is not there. The bin contains %s' % (request.item, request.from_bin, self.bin_contents[request.from_bin]))
                response.success = False
            else:
                # all seems to be valid, move the item from one bin to the other
                self.bin_contents[request.from_bin].remove(request.item)
                self.bin_contents[request.to_bin].append(request.item)
                rospy.loginfo('Item %s moved from %s to %s.' % (request.item, request.from_bin, request.to_bin))
                response.success = True
        self.writeJSON(self.bin_contents)

        rospy.set_param('/mission_plan/bin_contents', self.bin_contents)

        return response

    # =========================================================================
    def writeJSON(self, data):

        state = {'bin_contents':data}
        return json.dump(state, file('current_bin_state.json', 'w'), indent=4, separators=(',', ': '), sort_keys=True)
        #return json.dump(state, file('current_bin_state.json', 'w'))

    # =========================================================================
    def getNextItem(self, request):
        response = GetNextItemResponse()
        if self.mission_plan is None:
            rospy.logerr('Invalid mission plan.')
            # response = ['','','']
            return

        if len(self.mission_plan)>0:
            if self.task == 'pick_task':
                item_name = self.mission_plan[0]['item']
                item_bin = self.mission_plan[0]['bin']
            if self.task == 'stow_task':
                item_name = self.mission_plan[0]
                item_bin = 'bin_A'  # TODO put the right/emptiest bin in here

            response.id.data = self.getItemID(item_name)
            response.item.data = item_name
            response.bin.data = item_bin
            # print response
            # keep track of how often we attempt to pick one particular item
            self.keepTrackOfTrials(response)
        else:
            # response = ['','','']
            rospy.logwarn('Mission plan empty!')

        return response

    # =========================================================================
    def getBinContents(self, request):
        response = GetBinContentsResponse()
        if self.mission_plan is None:
            rospy.logerr('Invalid mission plan.')
            # response = ['','','']
            return

        if len(self.mission_plan)>0:
            # Unfortuneately std_msgs has no string array
            response.bin_contents = self.bin_contents[request.bin.data]
        else:
            # response = ['','','']
            rospy.logwarn('Mission plan empty!')

        return response

    # =========================================================================
    def getItemIDFromName(self, request):
        response = GetItemIDFromNameResponse()
        if self.mission_plan is None:
            rospy.logerr('Invalid mission plan.')
            # response = ['','','']
            return

        if len(self.mission_plan)>0:
            print response
            response.id.data = self.getItemID(request.item.data)
        else:
            # response = ['','','']
            rospy.logwarn('Mission plan empty!')

        return response

    # =========================================================================
                # print self.mission_plan[0]['item']
                # print "argh", self.mission_plan

    # =========================================================================
    def keepTrackOfTrials(self, response):
        key = response.bin.data + response.item.data
        if key not in self.trials:
            self.trials[key] = 1
        else:
            self.trials[key] += 1
        # check that we have not tried to pick that particular item max_trial times
        # if so, remove it from the list, so it is not attempted again
        if self.trials[key] >= self.max_trials:
            rospy.logwarn('Picking item %s from %s has been attempted %d times. Dropping it from the mission plan, not attempting again.' % (response.item.data, response.bin.data, self.max_trials))
            self.popItem(None)

    # =========================================================================
    def popItem(self, request):

        print self.mission_plan

        if self.mission_plan is None:
            rospy.logerr('Invalid mission plan.')
            response = ['','']
            return

        if len(self.mission_plan)>0:
            if self.task == 'stow_task':
                response = ['tote_contents', self.mission_plan[0]]
            else:
                response = [self.mission_plan[0]['bin'], self.mission_plan[0]['item']]

            del self.mission_plan[0]
        else:
            response = ['','']
            rospy.logwarn('Popped all items from the mission plan!')

        return response

    # =========================================================================
    def getItemID(self, item_name):
        item_names = self.readJSON("../mission_descriptions/item_id_mapping.json")
        if item_name in item_names: return item_names[item_name]
        rospy.logerr('Wrong item name provided: %s' % item_name)
        return 0

        if item_name not in item_names:
            rospy.logerr('Item name %s not found in item_id_mapping.json during itemID lookup!' % item_name)
            return -1
        else:
            return item_names[item_name]



# =============================================================================
# =============================================================================
# =============================================================================
if __name__ == "__main__":
    rospy.init_node('mission_plan')

    missionPlan = MissionPlan()

    rospy.spin()
