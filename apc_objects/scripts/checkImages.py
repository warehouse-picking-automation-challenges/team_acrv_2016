#! /usr/bin/env python

# Script to check if an image file is readable

from PIL import Image
import glob
image_list = []
for filename in glob.glob('/home/baxter/co/apc_ws/src/apc_objects/images/*.jpg'):
    im=Image.open(filename)
    image_list.append(im.copy())
    del im
