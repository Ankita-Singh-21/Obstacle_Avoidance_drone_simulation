#!/usr/bin/env python
# ROS python API

'''
Goal : To make drone follow a square path

Step:

1) arm
2) set mode : px4-offboard, ardupilot-guided
3) take off from origin and reach desire altitude
4) Go to (10,10), as a starting point for square path

   go_to_launch()

5) Then starting followinng square path in order (10,10) -> (10,-10) ->(-10,-10) -> (-10,10) -> (10,10) and so on

    square_path_loop()

6) keep on follow square path until user terminate 

'''
import rospy
from std_msgs.msg import String

# 3D point & Stamped Pose msgs
from geometry_msgs.msg import *
# import all mavros messages and services
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge
import math
from tf import transformations
import numpy
global state_of_drone
state_of_drone = 0


# Flight modes class
# Flight modes are activated using ROS services
class fcuModes:
    def __init__(self):
        pass

    def setTakeoff(self):
            rospy.wait_for_service('/mavros/cmd/takeoff')
            try:
                takeoffService = rospy.ServiceProxy('/mavros/cmd/takeoff', mavros_msgs.srv.CommandTOL)
                takeoffService(altitude = 2, latitude = 0, longitude = 0, min_pitch = 0, yaw = 0)
            except rospy.ServiceException, e:
                print "Service takeoff call failed: %s"%e

    def setArm(self):
        rospy.wait_for_service('mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService(True)
        except rospy.ServiceException, e:
            print "Service arming call failed: %s"%e

    def setDisarm(self):
        rospy.wait_for_service('mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService(False)
        except rospy.ServiceException, e:
            print "Service disarming call failed: %s"%e

    def setGUIDEDMode(self):
            rospy.wait_for_service('mavros/set_mode')
            try:
                flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
                flightModeService(custom_mode='GUIDED')
            except rospy.ServiceException, e:
                print "service set_mode call failed: %s. Offboard Mode could not be set."%e

    def setAutoLandMode(self):
            rospy.wait_for_service('/mavros/cmd/land')
            try:
                flightModeService = rospy.ServiceProxy('/mavros/cmd/land', mavros_msgs.srv.CommandTOL)
                flightModeService(altitude = 0, latitude = 0, longitude = 0, min_pitch = 0, yaw = 0)
            except rospy.ServiceException, e:
                print "service set_mode call failed: %s.  The vehicle cannot land"%e

# Controller class is for all callback function to store ros topic messages
class Controller:
    # initialization method
    def __init__(self):
        # Drone state
        self.state = State()
        # Instantiate a setpoints message
        self.sp = PositionTarget()
        # set the flag to use position setpoints and yaw angle
        self.sp.type_mask = int('010111111000', 2)
        # LOCAL_NED
        self.sp.coordinate_frame = 1



    def posCb(self,msg):

        global yaw, dist , local_pos 

        local_pos = Point()


        local_pos.x = msg.pose.position.x
        local_pos.y = msg.pose.position.y
        local_pos.z = msg.pose.position.z

        quaternion = [ msg.pose.orientation.x, msg.pose.orientation.y,
         msg.pose.orientation.z, msg.pose.orientation.w ]



        # transforming orientation from quaternion to euler
        euler = transformations.euler_from_quaternion(quaternion)
        #print(euler,"euler")

        # getting third element from 1D euler matrix
        yaw = euler[2] 
        print(yaw,"yaw")

        

    ## Drone State callback
    def stateCb(self, msg):
        self.state = msg

    def callback_1(self,data):
 
        # Used to convert between ROS and OpenCV images
        br = CvBridge()

        # Safe forward move publisher
        pub = rospy.Publisher('/new_topic',String,queue_size=1)
 
        # Output debugging information to the terminal
        #rospy.loginfo("receiving video frame")
   
        # Convert ROS Image message to OpenCV image
        current_frame = br.imgmsg_to_cv2(data, desired_encoding = "32FC1")
        msg = String()

        for i in range(80, 450 ) :
            for j in range(250,480) :
                
                if (current_frame[i][j] < 2 ):

                    msg.data = "True"
                    pub.publish(msg)
                    rospy.loginfo(" stop")
                else:
                    msg.data = "False"
                    rospy.loginfo("gf")
                    pub.publish(msg)
   
        # Display image
        cv2.imshow("camera", current_frame)
   
        cv2.waitKey(1)

    def callback_2(self,msg):

        global signal_data
        signal_data = msg.data
        rospy.loginfo(msg.data)
		

    
# Main function
def main():

    # initiate node
    rospy.init_node('setpoint_node', anonymous=True)

    # flight mode object
    modes = fcuModes()

    # controller object
    cnt = Controller()

    # ROS loop rate
    rate = rospy.Rate(20.0)

    # Subscribe to drone state
    rospy.Subscriber('mavros/state', State, cnt.stateCb)

    # Subscribe to drone's local position
    rospy.Subscriber('mavros/local_position/pose', PoseStamped, cnt.posCb)

    # Node is subscribing to the video_frames topic
    rospy.Subscriber("/depth_camera/depth/image_raw", Image, cnt.callback_1)

    # Velocity publisher
    vel_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel_unstamped', Twist, queue_size=10)

    pub = rospy.Subscriber('/new_topic',String,  cnt.callback_2)

    

    flag = 0
    global local_pos
    ALT_SP = 2

    global state_of_drone
    global signal_data
    

    def adjust_yaw():

        cnt = Controller()
        global signal_data

        vel_msg = Twist()

       

        if (signal_data == "False"):
            vel_msg.linear.x=-0.2
            vel_msg.linear.y=0
            vel_msg.linear.z = 0

            vel_pub.publish(vel_msg)

        elif (signal_data == "True"):
            vel_msg.linear.x=0.0
            vel_msg.linear.y=0
            vel_msg.linear.z = 0
            
            vel_pub.publish(vel_msg)
            
           


    

        

    # Make sure the drone is armed
    while not cnt.state.armed:
        modes.setArm()
        rate.sleep()

    print(cnt.state.mode)

    while cnt.state.mode != "GUIDED":
        modes.setGUIDEDMode()
        print(" guided mode set")


    # ROS main loop
    while not rospy.is_shutdown():

        print(" Ready for mission...")

        while(((math.floor(local_pos.z))<  ALT_SP) and flag == 0):
            modes.setTakeoff()
            print("taking off...")

            if (math.floor(local_pos.z)) == ALT_SP:
                flag=1
                print("Altitude reached")

        

        
        adjust_yaw()
        

        
    	rate.sleep()

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
