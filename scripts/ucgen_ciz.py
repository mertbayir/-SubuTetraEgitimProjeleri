#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import Twist
import time
from math import radians

def ucgen_ciz():
    rospy.init_node("ucgen_ciz")
    yayin = rospy.Publisher("cmd_vel",Twist,queue_size=10)
    hiz = Twist()

    for i in range(4):
        rospy.loginfo("kosedesiniz...")
        hiz.linear.x = 0.3  
        mesafe = 1.0
        yer_degistirme = 0.0
        t0 = rospy.Time.now().to_sec()

        while (yer_degistirme < mesafe):
            yayin.publish(hiz)
            t1 = rospy.Time.now().to_sec()
            yer_degistirme = hiz.linear.x * (t1 - t0)

        hiz.linear.x = 0.0
        yayin.publish(hiz)
        radsn = math.radians(120)  
        hiz.angular.z = radsn
        yayin.publish(hiz)
        donme_suresi =1.0
        time.sleep(donme_suresi)
        hiz.angular.z = 0.0
        yayin.publish(hiz)
        rospy.loginfo("Donuldu...")
        
ucgen_ciz()
