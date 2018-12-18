#!/usr/bin/env python
# license removed for brevity
import sys
import rospy

class Console_Format:
    
    def INFO(self, node_name=None, msg=None):
        MSG = None if node_name is "" else self.GREEN+"[{}]".format(node_name)
        MSG = MSG + self.NO_COLOR + msg +self.NO_COLOR
        rospy.loginfo(MSG)
        
    def DEBUG(self, node_name=None, msg=None):
        MSG = None if node_name is "" else self.BLUE+"[{}]".format(node_name)
        MSG = MSG + self.NO_COLOR + msg +self.NO_COLOR
        rospy.logdebug(MSG)
        
    def WARN(self, node_name=None, msg=None):
        MSG = None if node_name is "" else self.YELLOW+"[{}]".format(node_name)
        MSG = MSG + self.YELLOW + msg   +self.NO_COLOR
        rospy.logwarn(MSG)
        
    def ERR(self, node_name=None, msg=None):
        MSG = None if node_name is "" else self.RED+"[{}]".format(node_name)
        MSG = MSG + self.RED + msg  +self.NO_COLOR
        rospy.logerr(MSG)
        
    def FATAL(self, node_name=None, msg=None):
        MSG = None if node_name is "" else self.RED+"[{}]".format(node_name)
        MSG = MSG + self.RED + msg  +self.NO_COLOR
        rospy.logfatal(MSG)
        
    def __init__(self):
        self.NO_COLOR   = "\033[0m"
        self.BLACK      = "\033[30m"
        self.RED        = "\033[31m"
        self.GREEN      = "\033[32m"
        self.YELLOW     = "\033[33m"
        self.BLUE       = "\033[34m"
        self.MAGENTA    = "\033[35m"
        self.CYAN       = "\033[36m"
        self.LIGHTGRAY  = "\033[37m"