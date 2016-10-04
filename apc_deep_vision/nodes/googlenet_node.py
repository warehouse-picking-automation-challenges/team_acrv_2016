#!/usr/bin/env python

import numpy as np

import cv2

import rospy
from apc_msgs.srv import ClassifyRegions, ClassifyRegionsResponse
from apc_msgs.msg import ObjectProposals
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import caffeInterface
import time
# import scipy.stats
import scipy.misc

VISUAL2 = False
IM_SAVE = False
NUM_CLASSES = 40

class GoogLeNetClassifier:

    def doClassify(self,img):
        img_trans = self.GoogLeNet.transformer.preprocess(self.GoogLeNet.input_name, img)
        self.GoogLeNet.net.blobs[self.GoogLeNet.input_name].data[...] = img_trans
        return self.GoogLeNet.net.forward()['prob'][0,]

    def classify_service_callback(self, req):
        print "Request received"

        self.req = req
        self.specific_object = True

        # toggle classifying flag
        self.classifying = True
        while self.classifying:
            donothing = 0
        response = self.response

        self.classifying = False
        self.specific_object = False
        # print "Finished classifying object proposals"
        return response

    def classify_all_service_callback(self, req):
        print "Request received"

        self.req = req
        self.specific_object = False
        print len(req.proposals.segments)
        for seg in req.proposals.segments:
            print 'yoooooooooooooooooooooooooo\n'
            print seg.header
        if req.proposals.segments[0].header.frame_id == 'kinect2_rgb_optical_frame':
            self.single_classify = True
        else:
            print 'Maaattttee, the frame id is: ', req.proposals.segments[0].header.frame_id

        # toggle classifying flag
        self.classifying = True
        while self.classifying:
            donothing = 0
        response = self.response
        self.classifying = False
        self.specific_object = False
        self.single_classify = False
        # print "Finished classifying object proposals"
        return response

    def object_proposal_callback(self, msg):
        # print "Proposal callback"
        if self.classifying:
            return
        self.proposals = msg.bounding_boxes
        self.segments = msg.segments
        self.overlay = self.cv_bridge.imgmsg_to_cv2(msg.overlay, "bgr8")
        self.masks = msg.masks

    def image_callback(self, msg):
        # print "Image callback"
        if self.classifying:
            return
        self.image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")

    def __init__(self):

        self.classifying = False
        # start service
        classify_srv = rospy.Service(
            '/googlenet_node/classify_regions', ClassifyRegions, self.classify_service_callback)
        classify_all_srv = rospy.Service(
            '/googlenet_node/classify_all', ClassifyRegions, self.classify_all_service_callback)

        # subscribe to object proposals and image
        obj_proposal_subscriber = rospy.Subscriber(
            '/object_proposal_node/object_proposals', ObjectProposals, self.object_proposal_callback)
        image_subscriber = rospy.Subscriber(
            '/realsense/rgb/image_raw', Image, self.image_callback)

        # get params
        model_root = rospy.get_param(
            'model_root', '/home/apc/co/apc_ws/src/apc_deep_vision/models/')
        caffe_root = rospy.get_param(
            'caffe_root', '/home/apc/co/caffe/')
        snapshot_filename = rospy.get_param(
            'snapshot_filename', 'chris_iter_2500.caffemodel')
            # 'snapshot_filename', 'stowing_1.caffemodel')
            # 'snapshot_filename', 'stowing_augmented.caffemodel')
        class_txt_file = rospy.get_param(
            'class_txt_file', 'classes.txt')

        # CvBridge
        self.cv_bridge = CvBridge()
        # Load model
        caffe_model_file = model_root + 'deploy_picking.prototxt'
        # caffe_model_file = model_root + 'deploy_stowing.prototxt'
        caffe_pretrained_file = model_root + snapshot_filename
        mean_vec = np.array([112.817125, 110.58435, 110.7852])
        self.GoogLeNet = caffeInterface.CaffeNet(caffe_root, caffe_model_file, caffe_pretrained_file, caffe_mean_vec = mean_vec)
        self.classes = []
        # load classes
        with open(model_root + class_txt_file, 'r') as ins:
            for line in ins:
                self.classes.append(line[:-1])

        self.single_classify = False

        while not rospy.is_shutdown():
            if self.classifying:

                # accept passing of image, bounding boxes and pointcloud segments
                if self.req.image.data:
                    print "adding image, proposals and segments"
                    self.image = self.cv_bridge.imgmsg_to_cv2(self.req.image, "rgb8")
                if self.req.proposals.bounding_boxes:
                    self.proposals = self.req.proposals.bounding_boxes
                if self.req.proposals.segments:
                    self.segments = self.req.proposals.segments
                if self.req.proposals.masks:
                    self.masks = self.req.proposals.masks

                # instantiate response message and use request information
                response = ClassifyRegionsResponse()

                ids = [0]

                # object string id
                if self.specific_object:
                    object_id = self.req.object_id.data
                    # obtain idex relating to the object id string
                    object_no = self.classes.index(object_id)
                    ids = ids + [object_no]

                for obj in self.req.other_object_ids:
                    print obj
                    ids = ids + [self.classes.index(obj.data)]

                ids.sort()

                # return if no region proposals have been received from object_proposal
                if None in (self.proposals, self.segments, self.image):
                    print response.message.data
                    response.message.data = "Not all information available yet"
                    response.success.data = False
                    self.classifying = False
                    return response

                # initialise any empty array to store cnn clasification response
                # vectors
                all_responses = np.zeros((0, NUM_CLASSES))
                known_object_responses = np.zeros((0, NUM_CLASSES))
                known_object_responses_norm = np.zeros((0, NUM_CLASSES))

                if IM_SAVE:
                    timestamp = int(time.time())
                    path = '/home/apc/data/live_proposals/Image'
                    text_file = open(path+str(timestamp)+'.txt', "w")
                    mask_count = 0
                    cv2.imwrite(path+str(timestamp)+'.jpeg',self.image[:,:,::-1])


                try:
                    # loop through object proposals and store output vectors

                    for bb,seg,mask in zip(self.proposals,self.segments,self.masks):
                        print "Classifying Region No. ", bb.label

                        output = self.doClassify(self.image[bb.top_left.y:bb.bottom_right.y, bb.top_left.x:bb.bottom_right.x, :])
                        print 'Got passed the doClassify function'

                        # !!!!!!!!!!!!!!! only for stowing !!!!!!!!!!
                        print len(output)
                        print NUM_CLASSES
                        if len(output) < NUM_CLASSES:
                            output = np.insert(output ,0, 0.0000000001) # dump
                            output = np.insert(output ,1, 0.0000000001) # bones
                            output = np.insert(output ,16, 0.0000000001) # fitness
                        ################################################
                        print output

                        output_known_objects = output.copy()
                        for i in range(0,output_known_objects.shape[0]-1):
                            if i not in ids:
                                output_known_objects[i]=0
                        all_responses = np.append(all_responses,output.reshape(1,NUM_CLASSES),axis=0)
                        known_object_responses = np.append(known_object_responses,output_known_objects.reshape(1,NUM_CLASSES),axis=0)
                        known_object_responses_norm = np.append(known_object_responses_norm,(output_known_objects/output_known_objects[:].sum()).reshape(1,NUM_CLASSES),axis=0)

                        print 'here 1'
                        if IM_SAVE:
                            text_file.write("{} {} {} {} {}\n".format(self.classes[all_responses[-1,:][:].argmax()], str(bb.top_left.x), str(bb.top_left.y), str(bb.bottom_right.x), str(bb.bottom_right.y)))
                            try:
                                cv_image = self.cv_bridge.imgmsg_to_cv2(mask, "bgr8")
                            except CvBridgeError as e:
                                print e
                            cv2.imwrite(path+str(timestamp)+'_mask'+str(mask_count).zfill(4)+'.jpeg',cv_image)
                            mask_count = mask_count + 1

                        if VISUAL2:
                            print self.classes[all_responses[-1,:][:].argmax()]
                            print self.classes[known_object_responses[-1,:][:].argmax()]
                            print self.classes[known_object_responses_norm[-1,:][:].argmax()]
                            cv2.imshow(object_id, self.image[
                                       bb.top_left.y:bb.bottom_right.y, bb.top_left.x:bb.bottom_right.x, :])
                            cv2.waitKey(0)
                            cv2.destroyWindow(object_id)

                        print 'here 2'
                    if IM_SAVE:
                        text_file.close()

                    if self.specific_object:
                        print all_responses[:,object_no]
                        print all_responses[:,object_no].argmax()

                        region_no_all= all_responses[:,object_no][:].argmax()
                        region_no_known = known_object_responses[:,object_no][:].argmax()
                        region_no_known_norm = known_object_responses_norm[:,object_no][:].argmax()

                    if VISUAL2:
                        if self.specific_object:
                            bb = self.proposals[region_no_all]
                            bb2 = self.proposals[region_no_known]
                            bb3 = self.proposals[region_no_known_norm]
                            cv2.imshow(object_id, self.image[
                                       bb.top_left.y:bb.bottom_right.y, bb.top_left.x:bb.bottom_right.x, :])
                            cv2.waitKey(0)
                            cv2.destroyWindow(object_id)
                            cv2.imshow(object_id, self.image[
                                       bb2.top_left.y:bb2.bottom_right.y, bb2.top_left.x:bb2.bottom_right.x, :])
                            cv2.waitKey(0)
                            cv2.destroyWindow(object_id)
                            cv2.imshow(object_id, self.image[
                                       bb3.top_left.y:bb3.bottom_right.y, bb3.top_left.x:bb3.bottom_right.x, :])
                            cv2.waitKey(0)
                            cv2.destroyWindow(object_id)

                    # remove object_no and dump from ids
                    ids.pop(0)
                    other_item_confidences = all_responses.copy().transpose()
                    other_item_confidences = other_item_confidences / other_item_confidences.sum(axis=1).astype('float64').reshape(other_item_confidences.shape[0],1)


                    previously_seen = []
                    next_best = 1

                    if self.specific_object:
                        response.segment = self.segments[region_no_all]
                        response.segment.header.frame_id = "/camera_rgb_optical_frame"
                        response.segment.header.stamp = rospy.Time.now()
                        ids.pop(ids.index(object_no))
                        response.item_confidence = other_item_confidences[object_no,:][:].max()
                        previously_seen = previously_seen + [object_no]

                    if not self.single_classify:
                        other_item_ids = []
                        other_item_confidences_out = []
                        other_item_segments = []

                        for i in ids:
                            if i in previously_seen:
                                next_best = next_best + 1
                            else:
                                next_best = 1
                            previously_seen = previously_seen + [i]

                            temp = other_item_confidences[i,:][:].copy()
                            temp = temp.flatten()
                            temp.sort()
                            max_val = temp[-next_best]
                            ind = list(other_item_confidences[i,:][:]).index(max_val)
                            # append other item class
                            other_item_ids.append(self.classes[i])
                            # append confidence
                            other_item_confidences_out.append(max_val)
                            # append segment
                            other_item_segments.append(self.segments[ind])

                            other_item_segments[-1].header.frame_id = "/camera_rgb_optical_frame"

                            other_item_segments[-1].header.stamp = rospy.Time.now()

                    response.other_item_ids = other_item_ids
                    response.other_item_confidences = other_item_confidences_out
                    response.other_item_segments = other_item_segments
                    response.success.data = True

                    if self.single_classify:
                        response.other_item_confidences = list(all_responses.reshape(all_responses.shape[1]))
                        response.other_item_ids = self.classes

                        self.single_classify = False

                # catch exceptions and return failed response
                except Exception as e:
                    print e
                    print 'yooo 3'
                    print response.message.data
                    response.success.data = False
                    self.classifying = False
                self.response = response
                self.classifying = False
                self.specific_object = False

if __name__ == "__main__":
    rospy.init_node('googlenet_node')
    try:
        googlenet_node = GoogLeNetClassifier()
    except rospy.ROSInterruptException:
        pass
