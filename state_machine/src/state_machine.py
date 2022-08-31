#!/usr/bin/env python3  

import rospy
from state_mobile_robot import Mobile_Robot_Machine
from state_robotic_arm import Robotic_Arm_Machine
import sys
import parse_task

def av(table_f,table_t):
    table_from,table_to,object_from,object_to = parse_task.get_task(table_f,table_t)
    object_on_board = [[],[]]
    ind_on_board = 0
    right_step = 0
    zalupa = 0

    for i,table in enumerate(table_from):
        print('table index {}, table name {}'.format(i,table),'\n--------')
        if(len(object_from[i])==0):
            print('next table','\n--------')
            continue
        elif(not mobile_robot.to_table(table)):
            sys.exit()
        cur_object = object_from[i]
        for ob in cur_object:
            print('try grasp object {}'.format(ob),'\n--------')
            for i in range(5):
                if(not robotic_arm.grasp_object(ob)):
                    mobile_robot.move_robot('right',0.07)
                    right_step +=1
                else:
                    print('object {} is grasped'.format(ob),'\n--------')
                    robotic_arm.object_to_board(ind_on_board)
                    object_on_board[0].append(ob)
                    object_on_board[1].append(ind_on_board)
                    ind_on_board +=1
                    break
            if(not right_step == 0):
                mobile_robot.move_robot('left',0.07*right_step)
                right_step = 0
        mobile_robot.move_robot('back')
    
    for i,table in enumerate(table_to):
        print('table index {}, table name {}'.format(i,table),'\n--------')
        if(len(object_to[i])==0):
            print('next table','\n--------')
            continue
        elif(not mobile_robot.to_table(table)):
            sys.exit()
        cur_object = object_to[i]
        ind_on_table = 0
        mobile_robot.move_robot('right',0.07)
        for ob in cur_object:
            print('object {} to table in pose {}'.format(ob,ind_on_table),'\n--------')
            try:
                ind_object = object_on_board[0].index(ob)
            except ValueError:
                ind_object = -1
            if (ind_object == -1):
                print('cant find object {} on board'.format(ob),'\n--------')
                continue
            robotic_arm.object_from_board(object_on_board[1][ind_object])
            if(table == 't_5'):
                mobile_robot.move_robot('left',0.17)
                robotic_arm.object_to_down_table()
                mobile_robot.move_robot('right',0.10)
            elif(table == 't_2'):
                if(zalupa == 0):
                    mobile_robot.move_robot('left',0.07)
                    zalupa = 1
                robotic_arm.object_pricision_test(ob)
            else:
                robotic_arm.object_to_table(ind_on_table)
                ind_on_table +=1
        zalupa = 0
        mobile_robot.move_robot('back')

if __name__ == '__main__':
    rospy.init_node('state_machine')
    mobile_robot = Mobile_Robot_Machine()
    robotic_arm = Robotic_Arm_Machine()

    #mobile_robot.to_home()
    av('table_from1','table_to1')
    av('table_from2','table_to2')
    
    mobile_robot.to_exit()
