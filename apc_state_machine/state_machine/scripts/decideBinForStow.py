import rospy
import smach
import mission_plan.srv
import copy

_sizes = {
    # PERFECT}
    'expo_dry_erase_board_eraser':'small',
    'soft_white_lightbulb':'small',

    # GREAT
    'dove_beauty_bar':'small',
    'rawlings_baseball':'small',
    'scotch_duct_tape':'small',
    'staples_index_cards':'small',
    'scotch_bubble_mailer':'large',
    'folgers_classic_roast_coffee':'medium',
    'crayola_24_ct':'small',
    'jane_eyre_dvd':'medium',
    'laugh_out_loud_joke_book':'small',
    'kleenex_tissue_box':'medium',
    'i_am_a_bunny_book':'medium',

    # GOOD
    'kleenex_paper_towels':'large',
    'ticonderoga_12_pencils':'small',
    'peva_shower_curtain_liner':'large',
    'elmers_washable_no_run_school_glue':'small',
    'platinum_pets_dog_bowl':'small',
    'command_hooks':'small',

    # OKAY
    'dasani_water_bottle':'small',
    'creativity_chenille_stems':'large',
    'safety_first_outlet_plugs':'small',
    'up_glucose_bottle':'small',

    # AVERAGE
    'clorox_utility_brush':'medium',
    'dr_browns_bottle_brush':'large',
    'oral_b_toothbrush_green':'small',
    'oral_b_toothbrush_red':'small',

    # MILD
    'woods_extension_cord':'small',
    'rolodex_jumbo_pencil_cup':'small',
    'cool_shot_glue_sticks':'small',

    # HARD
    'cherokee_easy_tee_shirt':'small',
    'barkely_hide_bones':'small',

    'hanes_tube_socks':'large',

    'kyjen_squeakin_eggs_plush_puppies':'small',

    'fiskars_scissors_red':'small',

    # WHOA
    'cloud_b_plush_bear':'small',
    'womens_knit_gloves':'small',
    'easter_turtle_sippy_cup':'medium',
    'fitness_gear_3lb_dumbbell':'small'}


class DecideBinForStow(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'],
        input_keys=['next_item_to_pick'],
        output_keys=['next_item_to_pick'] # objects_in_bin
        )


    def getBestBin(self, userdata):
        # if item in hand is small, go for the largest bin of small items,
        # else the smallest bin

        # if we get a small object, put it in the largest draw full of small items
        # if other object or no only small bins, pick the smallest

        # CHECK and see if the size is greater than 5

        bins = copy.deepcopy(userdata['next_item_to_pick']['bin_contents'])

        print 'bin_contents = \n', bins

        # NOTE blacklist bin E
        del bins['bin_E']
        del bins['bin_A']
        del bins['bin_J']
        del bins['bin_G']
        del bins['bin_K']
        del bins['bin_L']

        # bins.remove(bins.keys().index('bin_E'))

        if _sizes[userdata['next_item_to_pick']['item']] == 'small':
            for k in sorted(bins, key=lambda k: len(bins[k])):
                if len(bins[k]) >= 5:
                    go_for_points = True
                    for item in bins[k]:
                        if _sizes[item] != 'small':
                            go_for_points = False
                    if go_for_points:
                        return k

            for k in sorted(bins, key=lambda k: len(bins[k])):
                if len(bins[k]) >= 5:
                    break
                if len(bins[k]) >= 3:
                    go_for_points = True
                    for item in bins[k]:
                        if _sizes[item] != 'small':
                            go_for_points = False
                    if go_for_points:
                        return k


        # sort bins based on n. of items
        for k in sorted(bins, key=lambda k: len(bins[k])):
            skip = False
            for item in bins[k]:
                if _sizes[item] == 'large':
                    skip = True
            if not skip:
                return k





    # ==========================================================
    def execute(self, userdata):

        response = self.getBestBin(userdata)
        userdata['next_item_to_pick']['bin'] = response
        # userdata['next_item_to_pick'] = {'tote_contents':response.tote_contents,'bin_contents':response.bin_contents,'bin':'tote'}
        # {'item':}
        return 'succeeded'
