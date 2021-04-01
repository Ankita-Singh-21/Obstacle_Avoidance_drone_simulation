from scipy.spatial.transform import Rotation as R
import pygame

#!/usr/bin/env python
import rospy
import numpy as np
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped, Point ,TwistStamped


# Arming Function
def setArm():
	rospy.wait_for_service('mavros/cmd/arming')
	try:
		armService = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
		armService(True)
	except rospy.ServiceException:
		print("Service arming call failed")

# function to set flight mode to GUIDED Mode
def setGuidedMode():
	rospy.wait_for_service('mavros/set_mode')
	try:
		flightModeService = rospy.ServiceProxy('mavros/set_mode', SetMode)
		flightModeService(custom_mode='GUIDED')
	except rospy.ServiceException:
		print("service set_mode call failed. Offboard Mode could not be set.")

# Function to start takeoff
def takeoff(takeoff_altitude=1):
	rospy.wait_for_service('mavros/cmd/takeoff')
	try:
		takeoffService = rospy.ServiceProxy('mavros/cmd/takeoff', CommandTOL)
		takeoffService(altitude = takeoff_altitude)
	except rospy.ServiceException:
		print(">> Takeoff Failed")

def land():
	rospy.wait_for_service('mavros/set_mode')
	try:
		flightModeService = rospy.ServiceProxy('mavros/set_mode', SetMode)
		flightModeService(custom_mode='LAND')
	except rospy.ServiceException:
		print("service set_mode call failed. Offboard Mode could not be set.")

state = State()
local_pos = PoseStamped()

def stateCb(msg):
	state.armed = msg.armed
	state.guided = msg.guided

def posCb(msg):

	global z
	r = R.from_quat([msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w])
	z = r.as_euler('xyz')[-1]
	# z = z[-1]
	#mat = np.array( np.cos(z)-np.sin(z) , np.sin(z)+np.cos(z) )

pygame.init()
pygame.display.set_mode((300,300))

def main():
	# initializing a node to communicate with the ROS Master
	rospy.init_node('takeoff_node', anonymous=True)

	#defining Scubcribers
	rospy.Subscriber('/mavros/state', State, stateCb)
	rospy.Subscriber('/mavros/local_position/pose', PoseStamped, posCb)
	sp_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
	vp_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
	rate = rospy.Rate(20) #10Hz

	# Arm the Drone
	while not state.armed:
		setArm()
		rate.sleep()

	# Set to GUIDED Mode
	setGuidedMode()

	# Start Takeoff
	takeoff(2)
	v = TwistStamped()
	while not rospy.is_shutdown():
		pygame.time.delay(10) 
		for event in pygame.event.get():
			if event.type == pygame.KEYDOWN:

				if event.key == pygame.K_RIGHT:	#x0y1
					v = TwistStamped()
					v.twist.linear.x = -np.sin(z)
					v.twist.linear.y = -np.cos(z)
					vp_pub.publish(v)
					print("Right",end='\r')

				if event.key == pygame.K_LEFT: #x0y-1
					v = TwistStamped()
					v.twist.linear.x = np.sin(z)
					v.twist.linear.y = np.cos(z)
					vp_pub.publish(v)
					print("Left",end='\r')

				if event.key == pygame.K_UP:	#x1y0
					v = TwistStamped()
					v.twist.linear.x = np.cos(z)
					v.twist.linear.y = np.sin(z)
					vp_pub.publish(v)
					print("forward",end='\r')

				if event.key == pygame.K_DOWN:	#x-1y0
					v = TwistStamped()
					v.twist.linear.x = -np.cos(z)
					v.twist.linear.y = -np.sin(z)
					vp_pub.publish(v)
					print("backward",end='\r')

				if event.key == pygame.K_w:
					v = TwistStamped()
					v.twist.linear.z=.5
					vp_pub.publish(v)
					print("Up",end='\r')

				if event.key == pygame.K_s:
					v = TwistStamped()
					v.twist.linear.z=-.5
					vp_pub.publish(v)
					print("Down",end='\r')

				if event.key == pygame.K_q:
					v = TwistStamped()
					v.twist.angular.z=.5
					vp_pub.publish(v)
					print("Up",end='\r')

				if event.key == pygame.K_e:
					v = TwistStamped()
					v.twist.angular.z=-.5
					vp_pub.publish(v)
					print("Down",end='\r')

				if event.key == pygame.K_SPACE:
					v = TwistStamped()
					vp_pub.publish(v)
					print("Stop",end='\r')

				if event.key == pygame.K_x:
					print("Quit",end='\r')
					pygame.quit()
					R = 0

			vp_pub.publish(v)



if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass