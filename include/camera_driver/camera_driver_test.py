#!/usr/bin/env python
# license removed for brevity
import os
import sys
sys.path.append(os.path.expanduser("~"))
current_folder = os.path.dirname(os.path.realpath(__file__))

from camera_driver import Camera_Driver

print("initializing ...")
cd = Camera_Driver()
cd.init()

print("previewing ...")
for i in range(100):
    cd.preview()

print("capturing ...")
cd.capture(current_folder + "/image.jpg")
