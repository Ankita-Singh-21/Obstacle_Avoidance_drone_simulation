#!/usr/bin/env python
import rospy 
import numpy as np
import cv2
import os
import time
import math
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from std_msgs.msg import String,Float32

from cv_bridge import CvBridge
from sensor_msgs.msg import Image

bridge_aruco = CvBridge()
aruco_detected = 0
aruco_point_flag = 0

velocity = TwistStamped()

pose = 0
def land():
	rospy.wait_for_service('mavros/set_mode')
	try:
		flightModeService = rospy.ServiceProxy('mavros/set_mode', SetMode)
		flightModeService(custom_mode='LAND')
	except rospy.ServiceException:
		print("service set_mode call failed. Offboard Mode could not be set.")

def arucocallback(data_aruco):
    global pose,pub,vel_aruco,velocity,aruco_detected,aruco_detected_pub
    cv_image = bridge_aruco.imgmsg_to_cv2(data_aruco, desired_encoding=data_aruco.encoding)
    #cv2.imshow("Down Cam",cv_image)
    #img_name = "opencv_frame_{}.jpg".format(data.header.seq)
    #cv2.imwrite(img_name, cv_image)
    Arucodict = {"Aruco_5x5":cv2.aruco.DICT_5X5_100}
    # change frame name 
    image = cv_image
    arucoParams = cv2.aruco.DetectorParameters_create()
    arucoDict = cv2.aruco.Dictionary_get(Arucodict["Aruco_5x5"])
    (corners, ids, rejected) = cv2.aruco.detectMarkers(image, arucoDict, parameters=arucoParams)
    print("arucoloop")
    # loop over the detected ArUCo corners
    if(len(corners) > 0) :
        k = 1
        aruco_detected_pub.publish(k)
        pub.publish("Marker ID : 0")
        # flatten the ArUco IDs list
        ids = ids.flatten()
        print(ids)
        print("yesssssssssssssss")
        for (markerCorner, markerID) in zip(corners, ids):
            # extract the marker corners (which are always returned in
            # # top-left, top-right, bottom-right, and bottom-left order)
            corners = markerCorner.reshape((4, 2))
            (topLeft, topRight, bottomRight, bottomLeft) = corners
            # convert each of the (x, y)-coordinate pairs to integers
            topRight = (int(topRight[0]), int(topRight[1]))
            bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
            bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
            topLeft = (int(topLeft[0]), int(topLeft[1]))
            # draw the bounding box of the ArUCo detection
            cv2.line(image, topLeft, topRight, (0, 255, 0), 2)
            cv2.line(image, topRight, bottomRight, (0, 255, 0), 2)
            cv2.line(image, bottomRight, bottomLeft, (0, 255, 0), 2)
            cv2.line(image, bottomLeft, topLeft, (0, 255, 0), 2)
            # compute and draw the center (x, y)-coordinates of the ArUco
            # marker
            cX = int((topLeft[0] + bottomRight[0]) / 2.0)
            cY = int((topLeft[1] + bottomRight[1]) / 2.0)
            cv2.circle(image, (cX, cY), 4, (0, 0, 255), -1)
            print(cX,cY)
            # draw the ArUco marker ID on the image
            cv2.putText(image, str(markerID),
            (topLeft[0], topLeft[1] - 15), cv2.FONT_HERSHEY_SIMPLEX,
            0.5, (0, 255, 0), 2)
            print("[INFO] ArUco marker ID: {}".format(markerID))
            ###############################
            cv2.circle(image,(320,240),4,(0,255,0),-1)
            cv2.line(image, (cX,cY),(320,240),(255,0,0),2)
            #publish the velocity command
            
            vel_mag = math.sqrt((cX-320)**2 + (cY - 240)**2)
            if( vel_mag >= 20 ) :
                velocity.twist.linear.x = -0.2*(cY - 240)/vel_mag
                velocity.twist.linear.y = -0.2*(cX - 320)/vel_mag
                velocity.twist.linear.z = 0
                vel_aruco.publish(velocity)
                print(velocity.twist.linear.x,velocity.twist.linear.y)
            elif( vel_mag >= 10 ) :
                velocity.twist.linear.x = -0.01*(cY - 240)
                velocity.twist.linear.y = -0.01*(cX - 320)
                velocity.twist.linear.z = 0
                vel_aruco.publish(velocity)
                print(velocity.twist.linear.x,velocity.twist.linear.y)
            # elif( vel_mag >= 5 ) :
            #     velocity.twist.linear.x = -0.01*(cY - 240)/vel_mag
            #     velocity.twist.linear.y = -0.01*(cX - 320)/vel_mag
            #     velocity.twist.linear.z = 0
            #     vel_aruco.publish(velocity)
            #     print(velocity.twist.linear.x,velocity.twist.linear.y)
            else :
                print("LANDING")
                land()
        #np.save("field_view1.npy",datapoints)
        # show the output image
    else:
        if(pose < 0.035):
           pub.publish("Marker ID : 0, Landed")
        else:
           pub.publish("Marker ID : none, looking for marker")
           print(pose)
        print(aruco_detected)

    cv2.imshow("Image", image)
    if cv2.waitKey(1) & 0xFF ==ord('q'):
        cv2.destroyAllWindows()
    print("endloop")

def posecallback(data):
	global pose
	pose = data.pose.position.z
	#print(pose)

def listener():
    global vel_aruco,arucomessage,velocity,aruco_detected_pub,pub
    rospy.init_node('arucocam_node', anonymous=True)
    rospy.Subscriber("/camera/color/image_raw", Image, arucocallback)
    # rospy.Subscriber("/depth_camera/depth/image_raw", Image, callback)
    rospy.Subscriber("/mavros/local_position/pose",PoseStamped,posecallback)
    vel_aruco = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size = 10)

    arucopoint = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size= 10)

    aruco_detected_pub = rospy.Publisher("/aruco/detected",Float32,queue_size=10)
    #arucomessage = rospy.Publisher("/aruco/message", String, queue_size = 10)
    #vel_aruco = rospy.Publisher("/mavros/local_position/velocity_local", TwistStamped, queue_size = 10)

    pub = rospy.Publisher('/aruco/message',String, queue_size= 10)

    rospy.spin()

if __name__ == '__main__':
	listener()
