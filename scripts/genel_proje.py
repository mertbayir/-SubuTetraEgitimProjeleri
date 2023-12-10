#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image

class NesneTespitHareket():

  def __init__(self):
        
        rospy.init_node("nesne_tespit_hareket")
        self.bridge = CvBridge()
        self.sub = rospy.Subscriber('/camera/rgb/image_raw',Image, self.goruntu)
        self.lazer = rospy.Subscriber('/scan',LaserScan, self.lazerCallBack)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.hareket = Twist()
        self.yol = None
        rospy.spin()
        
  def lazerCallBack(self,mesaj):
        sol_on = list(mesaj.ranges[0:9])
        sag_on = list(mesaj.ranges[350:359])
        on = sol_on + sag_on
        min_on = min(on)
        if min_on > 2.5 :
            self.yol = True
        else:
            self.yol = False

  def goruntu(self,mesaj):

        cv_img = self.bridge.imgmsg_to_cv2(mesaj, 'bgr8')
        hsv = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)
        alt_kirmizi = np.array([0,100,100])
        ust_kirmizi = np.array([10,255,255])
        maske = cv2.inRange(hsv,alt_kirmizi,ust_kirmizi)
        sonuc = cv2.bitwise_and(cv_img,cv_img,mask=maske)
        cv2.imshow('Maske Görüntü', maske)
        h, w, d = cv_img.shape
        cv2.imshow('Kırpılmış Maske Görüntü', maske)
        M = cv2.moments(maske)
        cv2.imshow(" Kamera Görüntü ",cv_img)
        cv2.waitKey(5)
        if self.yol == True:   
            if M['m00'] > 0:
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                cv2.circle(cv_img, (cx, cy), 20, (0,0,255), -1)                
                sapma = cx-w/2
                self.hareket.linear.x = 0.2
                self.hareket.angular.z = -sapma/100
                self.pub.publish(self.hareket)

            else:
                self.hareket.linear.x = 0.02
                self.hareket.angular.z = 0.4
                self.pub.publish(self.hareket)
        else:
            self.hareket.linear.x = 0.0
            self.hareket.angular.z = 0.0
            self.pub.publish(self.hareket)


NesneTespitHareket()
