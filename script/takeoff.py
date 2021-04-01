#!/usr/bin/env python
import rospy 
import numpy as np
import cv2
import os
import time
#ROS to OpenCv - we require a CVBridge
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
bridge = CvBridge()

#importing mavros_msgs services
from geometry_msgs.msg import TwistStamped
from mavros_msgs.srv import SetMode
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import CommandTOL


rospy.init_node('mavros_takeoff_python')
rate = rospy.Rate(10)

# Set Mode
print "\nSetting Mode"
rospy.wait_for_service('/mavros/set_mode')
try:
    change_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
    response = change_mode(custom_mode="GUIDED")
    rospy.loginfo(response)
except rospy.ServiceException as e:
    print("Set mode failed: %s" %e)

# Arm
print "\nArming"
rospy.wait_for_service('/mavros/cmd/arming')
try:
    arming_cl = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
    response = arming_cl(value = True)
    rospy.loginfo(response)
except rospy.ServiceException as e:
    print("Arming failed: %s" %e)

# Takeoff
print "\nTaking off"
rospy.wait_for_service('/mavros/cmd/takeoff')
try:
    takeoff_cl = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)
    response = takeoff_cl(altitude=2, latitude=0, longitude=0, min_pitch=0, yaw=0)
    rospy.loginfo(response)
except rospy.ServiceException as e:
    print("Takeoff failed: %s" %e)


#if __name__ == '__main__':
 #   listener()

