#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import os
import numpy

class LoadImage(object):

    def __init__(self):

        self.image_sub = rospy.Subscriber("/depth_camera/depth/image_raw", Image, self.camera_callback)
        self.bridge_object = CvBridge()

    def camera_callback(self, data):

        try:
            cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding = "32FC1")
        except CvBridgeError as e:
            print(e)

        print(cv_image.shape)
        for i in range(0, (cv_image.shape[1] ) ) :
    	
    	    for j in range(0, (cv_image.shape[0]) ) :
                if(cv_image[j][i] >3 and cv_image[j][i] < 10):
                    print("j", j,"i",i)
                    

        cv2.imwrite('drone_image.jpg', cv_image)
        cv2.imshow('image', cv_image)
        numpy.savetxt("fileeee.txt",cv_image)
        cv2.waitKey(0)

def main():
    load_image_object = LoadImage()
    rospy.init_node('load_image_node', anonymous = True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
