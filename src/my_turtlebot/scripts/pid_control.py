#!/usr/bin/env python
import rospy
import numpy as np
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion

class Server:
	temp = 1
	def __init__(self,gx,gy,sx,sy,data):
		self.goalx = gx
		self.goaly = gy
		self.startx = sx
		self.starty = sy
		self.pub = rospy.Publisher("/mobile_base/commands/velocity",Twist,queue_size = 10)
		self.data = data
		self.int_err_lin = 0
		self.int_err_ang = 0
		self.lin_vel = 0
		self.ang_vel = 0
		self.currentposex = 0
		self.currentposey = 0
		self.yaw = 0
	def odom_call(self,msg):
		self.currentposex = msg.pose.pose.position.x
		self.currentposey = msg.pose.pose.position.y
		self.x_o = msg.pose.pose.orientation.x
		self.y_o = msg.pose.pose.orientation.y
		self.z_o = msg.pose.pose.orientation.z
		self.w_o = msg.pose.pose.orientation.w
		self.quat_theta = [self.x_o,self.y_o,self.z_o,self.w_o]
		roll,pitch,yaw= euler_from_quaternion(self.quat_theta)
		self.yaw = yaw
		self.goal(self.yaw,self.currentposex,self.currentposey)
	def goaldist(self):
		return ((gx - sx)**2+(gy - sy)**2)**0.5
	def dist(self,c1,c2):
		return ((self.t1 - self.c1)**2+(self.t2 - self.c2)**2)**0.5
	def goal(self,yaw,x,y):
		try:
			try:
		
				theta_g = np.arctan2(self.data[Server.temp][1] - y,self.data[Server.temp][0] - x)
				errangular = theta_g - yaw
				if errangular >= np.pi:
					errangular = errangular - 2*np.pi
				if errangular <= -np.pi:
					errangular = errangular + 2*np.pi
				om = 0.8*errangular
				tw = Twist()
				tw.linear.x = 0
				tw.angular.z = om
				pub = rospy.Publisher("/mobile_base/commands/velocity",Twist,queue_size = 100)
				rospy.loginfo("Angle in degrees: %s, rotation velocity: %s, goal_theta: %s, errangular: %s",str(yaw*180/np.pi),str(tw.angular.z),str(theta_g*180/np.pi),str(errangular*180/np.pi))
				if abs(errangular)<= 0.03:
					print("---------------------")
					print("goal orientation reached")
					print("---------------------")
					tw.angular.z = 0
					tw.linear.x = 1.0*(((self.data[Server.temp][0] - x)**2 + (self.data[Server.temp][1] - y)**2)**0.5)
				if (((self.data[Server.temp][0] - x)**2 + (self.data[Server.temp][1] - y)**2)**0.5) <= 0.03:
					tw.angular.z = 0.8*(errangular)
					tw.linear.x = 0.0
					print("---------------------")
					print("goal pose reached")
					print("---------------------")

				if abs(errangular)<= 0.03 and (((self.data[Server.temp][0] - x)**2 + (self.data[Server.temp][1] - y)**2)**0.5) <= 0.03:
					
					Server.temp = Server.temp + 1
					print("temp = ",Server.temp)
				
				else:
					pass
				pub.publish(tw)
			except IndexError :
				print("path empty")
				rospy.signal_shutdown("Reason")
		except rospy.ROSException:
			print("Process interupted")
			pass
	def pid(self):
		p_lin = 0.08
		i_lin = 0.01
		p_ang = 0.6
		i_ang = 0.0
		k = Server.temp
		r = rospy.Rate(20)
		try:
			while not rospy.is_shutdown():
				print("temp length: ",k)
				if k == len(self.data):
					print("its over")
					rospy.signal_shutdown("reason")
				else:
					pass
				self.c1 = self.currentposex
				self.c2 = self.currentposey
				self.t1 = self.data[k][0]
				self.t2 = self.data[k][1]
				r.sleep()
		except rospy.ROSInterruptException:
			rospy.loginfo("Process Terminted\n")
			pass

	def sub_loop(self):
		rospy.Subscriber('odom',Odometry,self.odom_call)
		rospy.spin()


def main():
	targetlist = []
	rospy.init_node("my_control",disable_signals = False)
	sub_path = rospy.wait_for_message("/chatter",Path)
	for i in range(len(sub_path.poses)):
		targetlist.append([sub_path.poses[i].pose.position.x,sub_path.poses[i].pose.position.y])
	targetlist.reverse()
	print(targetlist)
	l = len(targetlist)
	'''for i in range(20):
		x = 0.05*(i+1)
		targetlist.append([x,x])
	l = len(targetlist)'''

	serve = Server(targetlist[l-1][0],targetlist[l-1][1],targetlist[0][0],targetlist[0][1],targetlist)
	serve.sub_loop()
	
if __name__ == "__main__":
	main()
	print("done")

