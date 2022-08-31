#!/usr/bin/env python3
import rospy
import tf
import numpy as np
from math import sin, cos
from inverse_problem_srv.srv import point_cmd,point_cmdResponse
from std_srvs.srv import SetBool,Empty
import socket

camera_address = ('192.168.55.1',19090)  #172.34.0.248
server_address = ('',19090)
sync_i = 0
sync_i_prec = 0
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(server_address)
delay = 0.1
home_pose = '90 0 170 0'
gripper = True
angle_arm_srv = rospy.ServiceProxy('/angle_robot/cmd_point',point_cmd)
angle_grip_srv = rospy.ServiceProxy('/angle_robot/gripper_cmd',SetBool)
angle_search = rospy.ServiceProxy('/angle_robot/searching_pose',Empty)

def convert_point_coordinate(point_cam):
    point_cam_ar = np.array(point_cam+[1000])/1000
    point_cam_ar[2] = point_cam_ar[2]+0.014
    listener.waitForTransform("world", "camera", rospy.Time(), rospy.Duration(4.0))
    (trans,rot) = listener.lookupTransform("world", "camera", rospy.Time(0))
    euler_rot = tf.transformations.euler_from_quaternion(rot)
    trans_matrix = tf.transformations.compose_matrix(angles = euler_rot, translate = trans)
    point_world = np.dot(trans_matrix,point_cam_ar)[0:-1]*1000
    return list(point_world)

def grasp_object(cmd):
    global sync_i
    angle_search()
    msg = 'c:'+str(sync_i)+':'+cmd.point
    sock.sendto(msg.encode(),camera_address)
    while True:
        data, address = sock.recvfrom(128)
        if not data == None:
            data = data.decode().split(':')
            if(data[0] == 'c' and int(data[1])==sync_i):
                if(not data[2] == 'None'):
                    data_list = data[2:]
                    point_in_cam = list(map(float,data_list))
                    sync_i +=1
                    break
                else:
                    return point_cmdResponse(False)
            if(not int(data[1])==sync_i):
                sock.sendto(msg.encode(),camera_address)
        rospy.sleep(delay)
    #print(point_in_cam)
    point_in_arm = convert_point_coordinate(point_in_cam[0:-1])
    point_in_arm = point_in_arm+[-point_in_cam[-1]]
    #print(point_in_arm)
    #point_in_arm[0] = point_in_arm[0] - 18*cos(point_in_arm[-1])
    #point_in_arm[1] = point_in_arm[1] - 18*sin(point_in_arm[-1])
    #print(point_in_arm)
    #print(point_in_arm)
    point_in_arm_str2 = list(map(str,point_in_arm))
    point_in_arm_str2 = ' '.join(point_in_arm_str2)
    point_in_arm[2] = point_in_arm[2]+30
    point_in_arm_str1 = list(map(str,point_in_arm))
    point_in_arm_str1 = ' '.join(point_in_arm_str1)
    res = angle_arm_srv(point_in_arm_str1)
    if(not res.result):
        print('unreachible1')
        return point_cmdResponse(False)
    rospy.sleep(3.0)
    res = angle_arm_srv(point_in_arm_str2)
    if(res.result):
        angle_grip_srv(False)
    else:
        print('unreachible2')
        return point_cmdResponse(False)
    #res = angle_arm_srv(point_in_arm_str1)
    return point_cmdResponse(True)

def precision_test(cmd):
    global sync_i_prec
    angle_search()
    obj = cmd.point
    if(obj == '4'):
        obj = '0'
    msg = 'p:'+str(sync_i_prec)+':'+obj
    sock.sendto(msg.encode(),camera_address)
    while True:
        data, address = sock.recvfrom(128)
        if not data == None:
            data = data.decode().split(':')
            if(data[0] == 'p' and int(data[1])==sync_i_prec):
                if(not data[2] == 'None'):
                    data_list = data[2:]
                    point_in_cam = list(map(float,data_list))
                    sync_i_prec +=1
                    break
                else:
                    return point_cmdResponse(False)
            if(not int(data[1])==sync_i_prec):
                sock.sendto(msg.encode(),camera_address)
        rospy.sleep(delay)
    point_in_arm = convert_point_coordinate(point_in_cam)
    point_in_arm = point_in_arm+[0]
    point_in_arm_str = list(map(str,point_in_arm))
    point_in_arm_str = ' '.join(point_in_arm_str)
    res = angle_arm_srv(point_in_arm_str)
    if(not res.result):
        print('unreachible1')
    rospy.sleep(3.0)
    angle_grip_srv(True)
    angle_arm_srv(home_pose)
    return point_cmdResponse(True)
    
if __name__=='__main__':
    global listener
    rospy.init_node('main_arm_control_node')
    listener = tf.TransformListener()
    rospy.wait_for_service('/angle_robot/cmd_point')
    angle_arm_srv(home_pose)
    angle_grip_srv(gripper)
    rospy.Service('/angle_robot/grasp_object',point_cmd,grasp_object)
    rospy.Service('/angle_robot/precision_test',point_cmd,precision_test)
    rospy.spin()

