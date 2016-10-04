#!/usr/bin/env python

import rospy
import json
import operator

_items = [
    # PERFECT
    'rawlings_baseball',
    'kleenex_tissue_box',
    'expo_dry_erase_board_eraser',
    'soft_white_lightbulb',

    # GREAT
    'folgers_classic_roast_coffee',
    'dove_beauty_bar',
    'up_glucose_bottle',
    'staples_index_cards',
    'crayola_24_ct',
    'jane_eyre_dvd',
    'laugh_out_loud_joke_book',
    'i_am_a_bunny_book',

    # GOOD
    'kleenex_paper_towels',
    'scotch_bubble_mailer',
    'ticonderoga_12_pencils',
    'peva_shower_curtain_liner',
    'elmers_washable_no_run_school_glue',
    'dasani_water_bottle',
    'command_hooks',

    # OKAY
    'hanes_tube_socks',
    'creativity_chenille_stems',
    'safety_first_outlet_plugs',

    # AVERAGE
    'clorox_utility_brush',
    'dr_browns_bottle_brush',
    'scotch_duct_tape',
    'oral_b_toothbrush_green',
    'oral_b_toothbrush_red',

    # MILD
    'cool_shot_glue_sticks',
    'woods_extension_cord',

    # HARD
    'easter_turtle_sippy_cup',
    'cherokee_easy_tee_shirt',
    'barkely_hide_bones',
    'kyjen_squeakin_eggs_plush_puppies',
    'fiskars_scissors_red',
    'rolodex_jumbo_pencil_cup',

    # WHOA
    'cloud_b_plush_bear',
    'womens_knit_gloves',
    'platinum_pets_dog_bowl',
    'fitness_gear_3lb_dumbbell']



# =========================================================================
def readJSON(filename):
    try:
        return json.load(file(filename, 'r'))
    except:
        rospy.logerr('Fatal Error: Could not read mission description from file %s' % filename)
        return None


# =========================================================================
def modifyWorkOrder(data):
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



# =========================================================================
if __name__ == '__main__':
    import argparse
    import ast

    parser = argparse.ArgumentParser()
    parser.add_argument("inputname", type=str,
                        help="filename to load the json order from")
    parser.add_argument("outputname", type=str,
                        help="filename to save the json order to")
    args = parser.parse_args()

    d = readJSON(args.inputname)
    if d is not None:
        modifyWorkOrder(d)
        with open(args.outputname, 'w') as f:
            json.dump(d, f, indent=4, separators=(',', ': '), sort_keys=True)
