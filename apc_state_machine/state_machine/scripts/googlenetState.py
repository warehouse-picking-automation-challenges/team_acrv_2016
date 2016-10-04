import rospy
import smach

import apc_msgs.srv
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2,Image
from cv_bridge import CvBridge
from mission_plan.srv import GetItemIDFromName,GetItemIDFromNameRequest
from termcolor import colored

class GooglenetState(smach.State):

    MIN_CONFIDENCE_THRESHOLD = 0.5
    _items = [

        # PERFECT
        'kleenex_tissue_box',
        'expo_dry_erase_board_eraser',
        'soft_white_lightbulb',

        # GREAT
        'dove_beauty_bar',
        'rawlings_baseball',
        'scotch_duct_tape',
        'staples_index_cards',
        'command_hooks',
        'folgers_classic_roast_coffee',
        'crayola_24_ct',
        'elmers_washable_no_run_school_glue',
        'up_glucose_bottle',
        'safety_first_outlet_plugs',
        'jane_eyre_dvd',
        'laugh_out_loud_joke_book',
        'i_am_a_bunny_book',

        # GOOD
        'woods_extension_cord',
        'dasani_water_bottle',
        'ticonderoga_12_pencils',
        'peva_shower_curtain_liner',
        'kleenex_paper_towels',
        'platinum_pets_dog_bowl',

        # OKAY
        'cool_shot_glue_sticks',
        'rolodex_jumbo_pencil_cup',
        'creativity_chenille_stems',

        # AVERAGE
        'dr_browns_bottle_brush',
        'oral_b_toothbrush_green',
        'oral_b_toothbrush_red',

        # MILD

        # HARD
        'clorox_utility_brush',
        'scotch_bubble_mailer',
        'cherokee_easy_tee_shirt',
        'barkely_hide_bones',

        'hanes_tube_socks',

        'kyjen_squeakin_eggs_plush_puppies',

        'fiskars_scissors_red',

        # WHOA
        'cloud_b_plush_bear',
        'womens_knit_gloves',
        'easter_turtle_sippy_cup',
        'fitness_gear_3lb_dumbbell']

    def __init__(self,action='in_bin'):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'],
            # input_keys=['next_item_to_pick','cloud_data','object_proposals','tote_contents','most_central'],
            input_keys=['next_item_to_pick','cloud_data','object_proposals','most_central'],
            output_keys=['googlenet','next_item_to_pick']
        )

        # wait for the services to appear
        self.action = action

        if self.action == 'in_bin':
            srv_name ='/googlenet_node/classify_regions'
        elif self.action in ['at_kinect','at_tote']:
            srv_name ='/googlenet_node/classify_all'


        if self.action == 'at_kinect':
            self.points_pub = rospy.Publisher('/at_kinect/points',PointCloud2)
            self.im_pub = rospy.Publisher('/at_kinect/region',Image)
            self.cv_bridge = CvBridge()

        self.waitForService(srv_name)
        if self.action == 'at_tote':
            self.points_pub = rospy.Publisher('/at_kinect/points',PointCloud2)
            self.im_pub = rospy.Publisher('/at_kinect/region',Image)
            get_item_srv_name = '/mission_plan/get_item_id_from_name'
            self.cv_bridge = CvBridge()
            self.waitForService(get_item_srv_name)


        # create a proxy to that service
        self.srv_googlenet = rospy.ServiceProxy(srv_name, apc_msgs.srv.ClassifyRegions)
        if self.action == 'at_tote':
            self.srv_get_item_id = rospy.ServiceProxy(get_item_srv_name, GetItemIDFromName)

    # ==========================================================
    def waitForService(self, service):

        rospy.loginfo('Waiting for service %s to come online ...' % service)
        try:
            rospy.wait_for_service(service, timeout=0.1)
        except:
            rospy.logerr('Service %s not available. Restart and try again.' % service)
            return False
        else:
            return True

    # ==========================================================
    def execute(self, userdata):

        if self.action is 'in_bin':
            result = self.execute_in_bin(userdata)
        elif self.action in ['at_kinect','at_tote']:
            result = self.execute_at_kinect(userdata)
        print result
        return result

    def execute_in_bin(self,userdata):
        try:
            userdata['googlenet'] = {}

            request = apc_msgs.srv.ClassifyRegionsRequest()
            request.object_id.data = userdata['next_item_to_pick']['item']
            contents = userdata['next_item_to_pick']['bin_contents']

            if request.object_id.data in contents:
                contents.pop(contents.index(request.object_id.data))

            for obj in contents:
                object_id = String()
                object_id.data = obj
                request.other_object_ids.append(object_id)

            request.proposals = userdata['object_proposals']
            request.image = userdata['cloud_data']['image']

            response = self.srv_googlenet.call(request)

            # TODO: Add a check for confidence !!!! Will require addition in
            # googlenet_node

            if response.success.data:

                # print confidence results for item classifications
                print userdata['next_item_to_pick']['item'], " confidence: ", response.item_confidence
                # print type(response.other_item_ids) = list
                # print type(response.other_item_confidences) = tuple hence change to list below
                for item, conf in zip(response.other_item_ids,list(response.other_item_confidences)):
                    print item, " confidence: ", conf

                userdata['googlenet'] = {'current_item':userdata['next_item_to_pick']['item'],
                                         'item_segment':response.segment,
                                         'item_confidence':response.item_confidence,
                                         'other_proposal_ids': response.other_item_ids,
                                         'other_item_confidences': list(response.other_item_confidences),
                                         'other_item_segments': response.other_item_segments}
                return 'succeeded'
            else:
                print 'response false'
                return 'failed'

        except:
            print 'exception'
            return 'failed'

    def execute_at_kinect(self,userdata):
        print "at_kinect"
        userdata['googlenet'] = {}
        request = apc_msgs.srv.ClassifyRegionsRequest()
        print 'Tote contents: ', userdata['next_item_to_pick']['tote_contents']

        for item in userdata['next_item_to_pick']['tote_contents']:
            s = String()
            s.data = item
            request.other_object_ids.append(s)

        if self.action is 'at_kinect':
            # print userdata['most_central'].segments.
            request.proposals = userdata['most_central']
            self.points_pub.publish(request.proposals.segments[0])
            bb = request.proposals.bounding_boxes[0]
            imMsg = userdata['cloud_data']['image']
            image = self.cv_bridge.imgmsg_to_cv2(imMsg,'bgr8')
            image_cropped = image[bb.top_left.y:bb.bottom_right.y,bb.top_left.x:bb.bottom_right.x]
            print 'Cropped image shape: ', image_cropped.shape
            imMsg = self.cv_bridge.cv2_to_imgmsg(image_cropped,encoding='rgb8')
            self.im_pub.publish(imMsg)
        elif self.action is 'at_tote':
            request.proposals = userdata['object_proposals']
            item_id_request = GetItemIDFromNameRequest()

        request.image = userdata['cloud_data']['image']

        try:
            print "GooglenetState: at_kinect/at_tote service call"
            response = self.srv_googlenet.call(request)

            if self.action is 'at_tote' and response.success.data:
                print "GooglenetState: Action is at_tote"

                other_item_confidences = list(response.other_item_confidences)

                ## START NEW CODE
                # tote contents priority sorting
                print "GooglenetState: Sorting tote contents based on priority"
                ind = []
                for item in userdata['next_item_to_pick']['tote_contents']:
                    ind.append(self._items.index(item))
                ind.sort()
                tote_contents_sorted = [self._items[i] for i in ind]
                print "GooglenetState: Sorted tote contents based on priority: ", tote_contents_sorted

                # identify proposal with highest priority
                print "GooglenetState: Processing googlenet response against item priority"
                for item in tote_contents_sorted:

                    # check item against the classified proposals
                    if item in response.other_item_ids:

                        # safety check, pretty sure this can't possibly happen in this for/if section
                        if item == 'dump':
                            print "Item is dump, pretty sure this can't happen but have added anyway"
                            continue

                        # obtain index of highest priority item
                        proposal_ids = [i for i, x in enumerate(response.other_item_ids) if x == item]

                        # check confidence greater than min conf threshold / number of times item appears in tote
                        # this deals with multiple instances of the same item in the tote
                        ## IF THIS CAUSES DRAMAS, USE THE COMMENTED LINE INSTEAD however this will likely cause high priority items with multiple instances in tote to be ignored
                        proposal_id = None
                        for ind in proposal_ids:
                            if other_item_confidences[ind] < self.MIN_CONFIDENCE_THRESHOLD / tote_contents_sorted.count(item):
                            # if other_item_confidences[proposal_id] < MIN_CONFIDENCE_THRESHOLD:
                                print "GooglenetState: Confidence for *", colored(item,'red'), "* with confidence", other_item_confidences[ind], "does not meet the threshold criteria of ", self.MIN_CONFIDENCE_THRESHOLD
                                continue
                            else:
                                proposal_id = ind
                                print "GooglenetState: Picking *", colored(item,'green'), "* with ", other_item_confidences[proposal_id], "confidence"
                                break
                        # if no instance of this item meet threshold, move on to next priority item
                        if proposal_id is None:
                            continue
                        print "GooglenetState: Picking ", item, " with ", other_item_confidences[proposal_id], "confidence"

                        ### THIS MIGHT BE USEFUL, monitor /at_kinect/points (have hijacked publisher)
                        # publish points segment deemed associated with highest priority item
                        print "GooglenetState: Publishing pointcloud of selected item"
                        # TODO uncomment
                        print type(response.other_item_segments[proposal_id])
                        print 'yoooooooooooooooo'
                        self.points_pub.publish(response.other_item_segments[proposal_id])

                        # get numerical item id for userdate
                        print 'proposal id = ', proposal_id
                        item_id_request.item.data = item
                        print "GooglenetState: Requesting numerical item id"
                        item_id_response = self.srv_get_item_id.call(item_id_request)

                        # process remaining information from googlenet, currently not used
                        print "GooglenetState: Processing remaining information from googlenet"
                        other_item_ids = response.other_item_ids
                        item_id = other_item_ids.pop(proposal_id)
                        item_confidence = other_item_confidences.pop(proposal_id)
                        segments = response.other_item_segments
                        item_segment = segments.pop(proposal_id)

                        # fill userdata
                        print "GooglenetState: Filling userdata"
                        userdata['next_item_to_pick']['id'] = item_id_response.id.data
                        # userdata['next_item_to_pick']['id'] = proposal_id
                        userdata['next_item_to_pick']['item'] = item
                        userdata['googlenet'] = {'current_item': item_id,
                                                 'item_segment': item_segment,
                                                 'item_confidence': item_confidence,
                                                 'other_proposal_ids': other_item_ids,
                                                 'other_item_confidences': other_item_confidences,
                                                 'other_item_segments': segments}

                        # return with identified high priority item
                        print "GooglenetState: Returning, start picking: ", colored(item,'green')
                        return 'succeeded'
                    else:
                        # if item not in googlenet response (not sure this)
                        print "GooglenetState: Could not identify ", colored(item,'red'), ", moving on to next item."

                # return failed if no objects identified, or if my code sux
                print "GooglenetState: No objects identified... or my code sux"
                return 'failed'
                ## END NEW CODE

                ## OLD CODE
                # max_conf = 0
                # for i, ids, conf in zip(range(0,len(response.other_item_ids)),response.other_item_ids,other_item_confidences):
                #
                #     # JAMES remove this hard exclude
                #     if ids in ['fitness_gear_3lb_dumbbell']:
                #         continue
                #     if conf > max_conf:
                #         max_conf = conf
                #         max_conf_id = ids
                #         segment = i
                #
                #
                #
                # other_item_ids = response.other_item_ids
                # other_item_ids.pop(segment)
                # other_item_confidences.pop(segment)
                # segments = response.other_item_segments
                # item_segment = segments[segment]
                # segments.pop(segment)
                #
                # print "Picking ", max_conf_id, " with ", max_conf, "confidence"
                #
                # item_id_request.item.data = max_conf_id
                # item_id_response = self.srv_get_item_id.call(item_id_request)
                #
                # if response.success.data:
                #     userdata['next_item_to_pick']['id'] = item_id_response.id.data
                #     userdata['googlenet'] = {'current_item': max_conf_id,
                #                              'item_segment': item_segment,
                #                              'item_confidence': max_conf,
                #                              'other_proposal_ids': other_item_ids,
                #                              'other_item_confidences': other_item_confidences,
                #                              'other_item_segments': segments
                #                             }
                #     userdata['next_item_to_pick']['item'] = max_conf_id
                #     return 'succeeded'
                # else:
                #     return 'failed'

            elif self.action is 'at_kinect' and response.success.data:
                print 'at kinect response'
                ordered = list(response.other_item_confidences)  # Force this to be a list because all the hacks
                print 'Not ordered confidences: ', ordered
                clss = response.other_item_ids
                print 'Every possible class just for fun: ', clss
                ordered.sort()
                print 'Ordered confidences: ', ordered
                for conf in ordered[::-1]:  # invert the confidence order (max in index 0)
                    print conf
                    cl = clss[list(response.other_item_confidences).index(conf)]
                    print cl
                    if cl in userdata['next_item_to_pick']['tote_contents']:
                        userdata['next_item_to_pick']['item'] = cl
                        return 'succeeded'
                return 'failed'
            else:
                print 'Not at at_kinect or at_tote'
                return 'failed'
        except Exception as e:
            print e
            print "GooglenetState: Failed on exception, have fun debugging"
            return 'failed'
