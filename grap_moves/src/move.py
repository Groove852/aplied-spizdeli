#!/usr/bin/env python3  
import rospy
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
from time import sleep, time
from srv_msg.srv import DistCmd, DistCmdResponse
import tf
class ParkController:
	def __init__(self):
		self.ranges = []
		self.min_angle = 0
		self.max_angle = 0
		self.angle_inc = 0
		self.sub_scan = rospy.Subscriber("scan_front", LaserScan, self.scan_cb)
		self.cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
		self.initial_pose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1)
		self.s = rospy.Service('move_robot', DistCmd, self.service_cb)
		self.ready = False
		self.step = 0
		self.step_time = time()
		self.initial_pose = PoseWithCovarianceStamped()
		self.send_initial_pose(0,0,0)
	def scan_cb(self, data):
		self.ready = True
		self.min_angle = data.angle_min
		self.max_angle = data.angle_max
		self.angle_inc = data.angle_increment
		self.ranges = data.ranges
	def send_initial_pose(self,x,y,theta):
		self.initial_pose.header.frame_id = 'map'
		self.initial_pose.header.stamp = rospy.Time.now()
		self.initial_pose.pose.pose.position.x = x
		self.initial_pose.pose.pose.position.y = y
		self.initial_pose.pose.pose.position.z = 0
		quat = tf.transformations.quaternion_from_euler(0, 0, theta)
		self.initial_pose.pose.pose.orientation.x = quat[0]
		self.initial_pose.pose.pose.orientation.x = quat[1]
		self.initial_pose.pose.pose.orientation.x = quat[2]
		self.initial_pose.pose.pose.orientation.x = quat[3]
		self.initial_pose.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 
											0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 
											0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
											 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
											  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
											  0.0, 0.0, 0.0, 0.0, 0.0, 0.06853892326654787]
		for i in range(0,10):
			self.initial_pose_pub.publish(self.initial_pose)
			sleep(0.1)
	def service_cb(self, req):
		# print(req.dist)
		if req.cmd == "front":
			while(self.parking_front(0) == False):
				sleep(0.1)
			self.speed_publisher(0,0,0)
			return DistCmdResponse(True)
		if req.cmd == "right":
			for i in range(int(req.dist*100)):
				self.parking_front(-0.1)
				sleep(0.1)
			self.speed_publisher(0,0,0)
			return DistCmdResponse(True)
		if req.cmd == "left":
			for i in range(int(req.dist*100)):
				self.parking_front(0.1)
				sleep(0.1)
			self.speed_publisher(0,0,0)
			return DistCmdResponse(True)
		if req.cmd == "back":
			for i in range(20):
				self.speed_publisher(-0.13,0,0)
				sleep(0.1)
			self.speed_publisher(0,0,0)
			return DistCmdResponse(True)

		return DistCmdResponse(False)
		
	def parking_front(self, y_speed):
		angle = self.min_angle
		i = 0
		left = [0,0]
		right = [0,0]
		middle = [0,0]
		ranges_left_right = []
		while angle <= self.max_angle:
			if angle <= -0.58 and angle >= -0.62:
				left[0]+=self.ranges[i]
				left[1]+=1
			if angle >= 0.58 and angle <= 0.62:
				right[0]+=self.ranges[i]
				right[1]+=1
			if angle >= -0.03 and angle <= 0.03:
				middle[0]+=self.ranges[i]
				middle[1]+=1
			if angle >= -0.9 and angle <= 0.9:
				ranges_left_right.append(self.ranges[i])
			i+=1
			angle+=self.angle_inc
		iter_left = 0
		iter_right = 0
		for i in range(0,len(ranges_left_right)-1,1): #need to find straight part of the wall
			if ranges_left_right[i] <= 0.2:
				iter_left = i
				break
		for i in range(len(ranges_left_right)-1,0,-1):
			if ranges_left_right[i] <= 0.2:
				iter_right = i
				break
		error_x = middle[0]/middle[1] - 0.08
		error_z = 0
		up_x = error_x
		if abs(up_x) <=0.03:
			error_z = ranges_left_right[len(ranges_left_right)-1] - ranges_left_right[0]
		up_z = 3*error_z
		# print(up_z)
		if abs(up_x) >= 0.15:
			up_x = self.sign(up_x)*0.15
		if abs(up_z) >= 0.1:
			up_z = self.sign(up_z)*0.1
		if abs(up_z) <= 0.05:
			up_z = 0
		self.speed_publisher(up_x, y_speed ,up_z)
		if abs(up_x) < 0.01 and abs(up_z) < 0.05:
			return True
		else:
			return False

	def sign(self, data):
		if data > 0:
			return 1
		else:
			return -1
	def speed_publisher(self,x,y,z):
		msg = Twist()
		msg.linear.x = x
		msg.linear.y = y
		msg.angular.z = z 
		self.cmd_pub.publish(msg)

if __name__ == '__main__':
	rospy.init_node('park_controller')
	controller = ParkController()
	sleep(1)
	rospy.spin()