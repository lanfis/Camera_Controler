#!/usr/bin/env python
# license removed for brevity
import os
import sys
sys.path.append(os.path.expanduser("~"))
current_folder = os.path.dirname(os.path.realpath(__file__))
include_folder = os.path.abspath(current_folder + "/../../include")
sys.path.append(current_folder)
sys.path.append(include_folder)
sys.path.append(current_folder + '/../matrix/python')
from console_format import Console_Format
OUT = Console_Format()

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import piggyphoto as pphoto



OUT = Console_Format()
class Camera_Driver:
    ##PUBLIC:
    node_name = "Camera_Controler"
    topic_image_pub = node_name + "/image_pub"
    topic_control_data_sub = node_name + "/control_data_sub"
    temp_folder = current_folder
    preview_data_name = "preview.jpg"
    image_data_name = "image.jpg"
    
    flag_window = False
    
    ##PRIVATE:
    camera_handler = pphoto.camera(autoInit=False)
    is_init = False
    image = None
    control_data = ""
    image_folder = ""
    image_name   = ""
    
    queue_size = 4
    bridge = CvBridge()
    image_pub_ = None
    control_data_sub_ = None
    
    def run(self):
        self.preview()
        self.image_publish()
    
    def abilities(self):
        return self.camera_handler.abilities
    
    def download(self, dst_path=current_folder, src_folder=None, src_name=None):
        src_folder = self.image_folder if src_folder is None else src_folder
        src_name = self.image_name if src_folder is None else src_name
        
        OUT.INFO(self.node_name, "Downloading images : {} ...".format(os.path.join(src_folder, self.src_name)))
        self.camera_handler.download_file(srcfolder, srcfilename, dst_path)
    
    def preview(self, dst_path=None):
        if not self.is_init:
            self.init()
        path = os.path.join(self.temp_folder, self.preview_data_name) if dst_path is None else dst_path
        self.camera_handler.capture_preview(path)
        self.image = cv2.imread(path, cv2.IMREAD_COLOR)
        return path
        
    def capture(self, download_path=None):
        path = download_path
        OUT.INFO(self.node_name, "Capturing image ...")
        if download_path is None:
            self.image_folder, self.image_name = self.camera_handler.capture_image()
            path = os.path.join(self.image_folder, self.image_name)
        else:
            self.camera_handler.capture_image(destpath=download_path)
        OUT.INFO(self.node_name, "Capturing image ok ! Saving data in : {}".format(path))
    
    def exit(self):
        self.camera_handler.exit()

    def image_publish(self):
        self.image_pub_.publish(self.bridge.cv2_to_imgmsg(self.image, "bgr8"))
        if self.flag_window:
            cv2.imshow(self.node_name, self.image)
            cv2.waitKey(1)
        
    def control_data_callback(self, data):
        self.control_data = data.data
        
        if self.control_data == "preview":
            self.preview()
            self.image_publish()

    def pub_init(self):
        OUT.INFO(self.node_name, "Publisher {} initiating !".format(self.topic_image_pub))
        self.image_pub_ = rospy.Publisher(self.topic_image_pub, Image, queue_size=self.queue_size)
        
    def sub_init(self):
        OUT.INFO(self.node_name, "Subscriber {} initiating !".format(self.topic_control_data_sub))
        self.control_data_sub = rospy.Subscriber(self.topic_control_data_sub, String, self.control_data_callback)

    def init(self):
        if self.is_init:
            OUT.INFO(self.node_name, "Reinitializing camera handler ...")            
            self.camera_handler.reinit()
        else:
            OUT.INFO(self.node_name, "Initializing camera handler ...")
            self.camera_handler.init()
            
        self.is_init = self.camera_handler.initialized
        if self.is_init:
            OUT.INFO(self.node_name, "Initializing camera handler ok !")
            print(self.abilities())
        else:
            OUT.WARN(self.node_name, "Initializing camera handler fail !")

    def __init__(self, name=None):
        self.node_name = self.node_name if name is None else name
        self.pub_init()
        self.sub_init()
        self.init()
        

