#!/usr/bin/env python
import rospy
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
global roll
global pitch
global yaw

def callbac(msg):
	
	
	x_o = msg.pose.pose.orientation.x
	y_o = msg.pose.pose.orientation.y
	z_o = msg.pose.pose.orientation.z
	w_o = msg.pose.pose.orientation.w

	quat_theta = [x_o,y_o,z_o,w_o]
	roll,pitch,yaw = euler_from_quaternion(quat_theta)
	#rospy.loginfo("Angle: %s",str(yaw*180/np.pi))
	
	#print(yaw)
	theta_g = np.arctan2(goal_y,goal_x)
	
	om = 1.0*(theta_g - yaw)
	tw = Twist()
	tw.linear.x = 0
	tw.angular.z = om
	pub = rospy.Publisher("/mobile_base/commands/velocity",Twist,queue_size = 100)
	pub.publish(tw)
	rospy.loginfo("Angle in degrees: %s, rotation velocity: %s",str(yaw*180/np.pi),str(tw.angular.z))
	if yaw == theta_g:
		print("goal reached")
		tw.angular.z = 0
	else:
		pass
	

	



	
		



def main():
	global goal_x
	global goal_y
	goal_x = int(input("Enter goal_x in degrees: "))
	goal_y = int(input("Enter goal_y in degrees: "))
	rospy.init_node("pid_rotation",disable_signals = True)
	rospy.Subscriber("odom",Odometry,callbac)

	rospy.spin()
if __name__ == "__main__":
	main()



