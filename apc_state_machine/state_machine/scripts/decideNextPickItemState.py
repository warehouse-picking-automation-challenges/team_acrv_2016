import rospy
import smach
import smach_ros
import threading
from std_msgs.msg import String
import mission_plan.srv


class DecideNextPickItemState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted', 'finished'],
        output_keys=['next_item_to_pick'] # objects_in_bin
        )

        # wait for the service to appear
        rospy.loginfo('Waiting for services of mission_plan/get_next_item to come online ...')
        try:
            rospy.wait_for_service('/mission_plan/get_next_item', timeout=1)
        except:
            rospy.logerr('Services of /mission_plan/get_next_item not available. Restart and try again.')
            # rospy.signal_shutdown('')


        self.get_next_item_service = rospy.ServiceProxy('/mission_plan/get_next_item', mission_plan.srv.GetNextItem)
        self.get_bin_contents_service = rospy.ServiceProxy('/mission_plan/get_bin_contents', mission_plan.srv.GetBinContents)
        self.get_item_id_from_name_service = rospy.ServiceProxy('/mission_plan/get_item_id_from_name', mission_plan.srv.GetItemIDFromName)

    # ==========================================================
    def execute(self, userdata):

        # try to call the service
        # !! warning, this could potentially block forever
        try:
            response = self.get_next_item_service.call()
            # print response
            print 'Aiming for: ' + '\033[92m' + '\033[1m' + response.item.data + '\033[0m'


            # Grab the bin contents
            string_msg = String()
            string_msg.data = response.bin.data
            bin_contents_response = self.get_bin_contents_service.call(string_msg)

            # Grab the bin contents ids
            bin_contents_ids_list = []
            for item in bin_contents_response.bin_contents:
                string_msg.data = item
                item_id = self.get_item_id_from_name_service.call(string_msg)
                bin_contents_ids_list.append(item_id.id.data)
            # print type(bin_contents_response) = <class 'mission_plan.srv._GetBinContents.GetBinContentsResponse'>
            # print type(bin_contents_response.bin_contents) = <type 'list'>


            rospy.loginfo('Decided next item to pick is %s from %s' % (response.item.data, response.bin.data))
            userdata['next_item_to_pick'] = {'bin':response.bin.data, 'item':response.item.data, 'id':response.id.data, 'bin_contents':bin_contents_response.bin_contents, 'bin_contents_ids':bin_contents_ids_list}
            if response.bin.data == '' or response.item.data == '' or response.id.data == 0:
                rospy.logwarn('Mission plans seems to be empty.')
                return 'finished'
            else:
                return 'succeeded'
        except Exception,e:
            rospy.logerr('Service call to mission_plan failed. Reason: \n %s' % e)
            return 'aborted'
