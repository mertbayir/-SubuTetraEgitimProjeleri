#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class carpmadanDur:
    def __init__(self):
        rospy.init_node('carpmadan_dur')
        self.laser_scan = LaserScan()
        self.hiz = Twist()
        self.hiz_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.laser_scan_subscriber = rospy.Subscriber('/scan', LaserScan, self.lazerCallback)

    def lazerCallback(self, scan):
        sol_on = list(scan.ranges[0:9])
        sag_on = list(scan.ranges[350:359])
        on = sol_on + sag_on
        sol = list(scan.ranges[80:100])
        sag = list(scan.ranges[260:280])
        min_on = min(on)
        min_sol = min(sol)
        min_sag = min(sag)
    
        if min_sol > 1.0 and min_sag >1.0 and min_on > 1.0: 
                self.hiz.linear.x = 0.3
                self.hiz.angular.z = 0.0
                self.hiz_publisher.publish(self.hiz)
                rospy.loginfo('Hareket Ediliyor...')

        else:
                self.hiz.linear.x = 0.0
                self.hiz.angular.z = 0.0
                self.hiz_publisher.publish(self.hiz)
                rospy.loginfo('Cisim Var, Devam Edilemiyor..!')
            
               
if __name__ == '__main__':
    try:
        obstacle_avoider = carpmadanDur()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo('Geçersiz İşlem...')
  
        
        
        
        
        
        
        
