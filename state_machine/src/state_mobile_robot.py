#!/usr/bin/env python3  

import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction
from move_base_msgs.msg import MoveBaseActionGoal
from move_base_msgs.msg import MoveBaseGoal
from move_base_msgs.msg import MoveBaseActionFeedback
from move_base_msgs.msg import MoveBaseActionResult
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from srv_msg.srv import DistCmd, DistCmdResponse
from nav_msgs.srv import GetMap, GetPlan
from std_msgs.msg import Bool
import tf
from math import pi, cos, sin, sqrt, atan2
import numpy as np
import json
import os
class Mobile_Robot_Machine:
	def __init__(self):
		self.client_move_base = actionlib.SimpleActionClient('move_base',MoveBaseAction)
		self.client_move_base.wait_for_server()
		self.move_robot_srv = rospy.ServiceProxy('move_robot', DistCmd)
		self.table_coordinates = self.get_tables_coordinates()
		# print(self.table_coordinates)
		# self.to_home()
		# print(self.table_coordinates['t_2'])
		# print(self.to_table("t_6"))
		# print(self.move_robot("front"))
		
		# print(self.move_robot("right",0.1))
		# print(self.move_robot("left",0.1))
		# print(self.move_robot("back"))
		# print(self.reach_goal(self.table_coordinates['t_7']))
		# print(self.reach_goal(self.table_coordinates['home']))
		
	def to_table(self, table):
		result = self.reach_goal(self.table_coordinates[table])
		if result == 3:
			result = self.move_robot("front")
			return result
		else:
			return False
	def to_home(self):
		result = self.reach_goal(self.table_coordinates["home"])
		if result == 3:
			return True
	def to_exit(self):
		result = self.reach_goal(self.table_coordinates["exit"])
		if result == 3:
			return True
	
	def get_tables_coordinates(self):
		dir_path = os.path.dirname(os.path.realpath(__file__))	
		with open(dir_path+"/points.json", "r") as read_file:
			data = json.load(read_file)
		table_coord = dict(zip(data["table_id"], data["table_coordinate"]))	
		return table_coord

	def move_robot(self, movement, length=0.0): #front, left, right, back, length of side transmition
		resp = self.move_robot_srv(movement, length)
		return resp.result

	def reach_goal(self, coord): #go to target point 
		x = coord[0]
		y = coord[1]
		theta = coord[2]
		target_quat = tf.transformations.quaternion_from_euler(0, 0, theta)
		t0=rospy.Time.now()
		goal=MoveBaseGoal()
		goal.target_pose.header.stamp=t0
		goal.target_pose.header.frame_id="map"
		goal.target_pose.pose.position = Point(x, y, 0)
		goal.target_pose.pose.orientation.x = target_quat[0]
		goal.target_pose.pose.orientation.y = target_quat[1]
		goal.target_pose.pose.orientation.z = target_quat[2]
		goal.target_pose.pose.orientation.w = target_quat[3]  
		self.client_move_base.send_goal(goal)
		reached_the_goal = self.client_move_base.wait_for_result(rospy.Duration.from_sec(60))
		if not reached_the_goal:
			rospy.logwarn("I was not able to reach the goal within the\
					allocated time")
			self.client_move_base.cancel_goal()    
		else:
			print("reach the goal","\n--------")
		return self.client_move_base.get_state()


# if __name__ == '__main__':
	# rospy.init_node('state_machine')
	# state = Mobile_Robot_Machine()
	# try:
		# rospy.spin()
	# except KeyboardInterrupt:
		# print("Shutting down")

        
