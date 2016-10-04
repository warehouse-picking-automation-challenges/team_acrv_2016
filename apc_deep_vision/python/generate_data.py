#! /usr/bin/env python
# ********************************************************************
# Software License Agreement (BSD License)
#
#  Copyright (c) 2015, University of Colorado, Boulder
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#   * Neither the name of the University of Colorado Boulder
#     nor the names of its contributors may be
#     used to endorse or promote products derived
#     from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.
# ********************************************************************/

import cv2
import os
import numpy as np


if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("proposal_path", type=str,
                        help="relative path from python script to proposals, no slash")
    parser.add_argument("--view", default=None,
                        help="true/1 shows each masked image")
    args = parser.parse_args()


    # args.proposal_path = "../test_proposals"
    # args.proposal_path = args.proposal_path

    included_extenstions = ['txt']
    image_names = [fn[0:len(fn)-4] for fn in os.listdir(args.proposal_path)
                    if any(fn.endswith(ext) for ext in included_extenstions)]

    for image_name in image_names:
        load_path = args.proposal_path + '/' + image_name

        image = cv2.imread(load_path + ".jpeg")
        data = np.loadtxt(load_path + ".txt", str)

        # If there is only one line, force data to be a list of lists anyway
        # Note, only works for our data as first list item is a string
        if isinstance(data[0], basestring):
            data = [data]

        # If any line does not conform to classification tl_x tl_y br_x br_y
        # then forget about it
        skip = False
        for line in data:
            if len(line) < 5:
                skip = True
        if skip:
            continue

        for i, proposal in zip(range(0,len(data)),data):
            mask = cv2.imread(load_path + '_mask{0:04d}.jpeg'.format(i))
            mask = np.invert(mask)
            maskGray = cv2.cvtColor(mask, cv2.COLOR_BGR2GRAY)
            ret, maskGray = cv2.threshold(maskGray,128,255,cv2.THRESH_BINARY)

            print load_path + '_mask{0:04d}.jpeg'.format(i)
            cropped = image[float(proposal[2]):float(proposal[4]), float(proposal[1]):float(proposal[3])]

            masked = cv2.bitwise_and(cropped, cropped, mask = maskGray)

            if args.view:
                cv2.imshow("original", masked)
                cv2.waitKey(0)

            mask_directory = args.proposal_path + '/masked/' + proposal[0];
            crop_directory = args.proposal_path + '/cropped/' + proposal[0];
            if not os.path.exists(mask_directory):
                os.makedirs(mask_directory)
            if not os.path.exists(crop_directory):
                os.makedirs(crop_directory)

            cv2.imwrite(mask_directory + '/{}_{}.jpeg'.format(image_name,i), masked)
            cv2.imwrite(crop_directory + '/{}_{}.jpeg'.format(image_name,i), cropped)








    # item = data[]
    # cropped = image[70:170, 440:540]
                # startY:endY, startX:endX
                # startX:startY, endX:endY


#
