import rospy
import smach
import geometry_msgs.msg

class GetBinCoordinatesState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'],
        input_keys=['next_item_to_pick'],
        output_keys=['goal_frame_id', 'goal_pose']
        )

        try:
            self.shelf_layout = rospy.get_param('shelf_layout')
            # print "testing the shelf height: ", self.shelf_layout['bin_height']\
            print "testing the shelf height: ", self.shelf_layout['bin_A']
            print "testing the shelf height of D: ", self.shelf_layout['bin_D']['bin_height']
        except:
            rospy.logerr('Could not load bin_coordinates parameter!')

    # ==========================================================
    def execute(self, userdata):

        requested_bin = userdata['next_item_to_pick']['bin']
        bin = self.shelf_layout[requested_bin]

        pose = geometry_msgs.msg.PoseStamped()
        # this seems to be the correct rotation / so don't TOUCH IT!!
        pose.pose.orientation.x = 0
        pose.pose.orientation.y = -0.707
        pose.pose.orientation.z = 0
        pose.pose.orientation.w = 0.707
        # pose.pose.position.x += 0.22
        # pose.pose.position.y -= 0.15
        # pose.pose.position.z -= 0.30

        pose.pose.position.x += bin['bin_height'] / 3  # should be higher up than half for good measures
        pose.pose.position.y -= bin['bin_width'] / 2
        pose.pose.position.z -= self.shelf_layout['bin_obs_distance']

        pose.header.frame_id = requested_bin
        pose.header.stamp = rospy.Time.now()

        userdata['goal_frame_id'] = requested_bin
        userdata['goal_pose'] = pose


        # print "\n\n\n\n", "%s_start_pose: position" % requested_bin
        # print pose.pose.position
        # print "\n\n\n\n"

        return 'succeeded'



        # if not requested_bin in self.shelf_layout:
        #     rospy.logerr('Unknown goal frame %s when determining bin position in shelf.' % requested_bin)
        #     return 'aborted'
        # else:
