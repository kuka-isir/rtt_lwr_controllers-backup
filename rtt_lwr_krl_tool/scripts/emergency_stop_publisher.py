#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Sat Jul  9 15:20:22 2016

@author: hoarau
"""

import rospy
from std_srvs.srv import Empty
from std_msgs.msg import Float32
from time import time
from krl_msgs.srv import SetMaxVelPercent

rospy.init_node("safety_stop")

stop_zone = 0.4 # 1.4m
hysteresis = 0.2


#hysteresis_time = 1.0 # 1s
#oldest_ok = -1
max_dist  = 2.0

def distanceCB(msg):
    d = msg.data
    if d > max_dist:
        d = 2.0
    d = d/max_dist*100

    vel_srv(d)

    # print msg
    if msg.data < stop_zone :
        # print "STOP"
        stop2_srv()
    elif msg.data < stop_zone + hysteresis :
        pass
        # print "Neutral zone"
    else :
        # print 'OK'
        unset_stop2_srv()


rospy.wait_for_service("/lwr_krl_tool/set_max_vel_percent")
rospy.wait_for_service("/lwr_krl_tool/send_stop2")
rospy.wait_for_service("/lwr_krl_tool/unset_stop2")

dist_sub = rospy.Subscriber("/sick_proc/min_dist_to_laser",Float32,callback=distanceCB,tcp_nodelay=True,queue_size=10)
vel_srv = rospy.ServiceProxy("/lwr_krl_tool/set_max_vel_percent",SetMaxVelPercent)
stop2_srv = rospy.ServiceProxy("/lwr_krl_tool/send_stop2", Empty)
unset_stop2_srv = rospy.ServiceProxy("/lwr_krl_tool/unset_stop2", Empty)

rospy.spin()
print("End")
exit()
