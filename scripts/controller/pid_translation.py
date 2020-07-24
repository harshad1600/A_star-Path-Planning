#!/usr/bin/env python
import rospy
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
k = 4

def callbac(msg):
	x = msg.pose.pose.position.x
	#y = msg.pose.pose.position.y
	twist = Twist()
	twist.linear.x = k*(goal_x - x)
	print(twist.linear.x)
	pub = rospy.Publisher("/mobile_base/commands/velocity",Twist,queue_size = 100)
	rospy.loginfo("Position: %s",str(x))
	pub.publish(twist)
	

def main():
	
	rospy.init_node("pid_translation")
	
	rospy.Subscriber("odom",Odometry,callbac)
	rospy.spin()
if __name__ == "__main__":
	global goal_x
	#global goal_y
	goal_x = int(input("Enter goal x: "))
	#goal_y = int(input("Enter goal y: "))
	main()
