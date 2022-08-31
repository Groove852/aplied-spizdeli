#!/usr/bin/env python3
import rospy

rospy.init_node('test_time')

t1 = rospy.get_time()
rospy.sleep(1)
print(rospy.get_time()-t1)