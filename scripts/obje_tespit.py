#!/usr/bin/env python3
# -*- coding: utf-8 -*-


import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from find_object_2d.msg import ObjectsStamped

class Kamera():
    def __init__(self):
        rospy.init_node("kamera_dugumu")
        self.bridge = CvBridge()
        rospy.Subscriber("camera/rgb/image_raw",Image,self.kameraCallback)
        rospy.Subscriber("objectsStamped",ObjectsStamped,self.kameraCallback)
        rospy.spin()
        
    def kameraCallback(self,mesaj):
        img = self.bridge.imgmsg_to_cv2(mesaj,"mono8")
        img2 = self.bridge.imgmsg_to_cv2(mesaj,"bgr8")
        self.nesne_id = mesaj.objects.data[0]
        a,esiklenmis = cv2.threshold(img,127,255,cv2.THRESH_BINARY_INV)
        sinirlar,hiyerarsi = cv2.findContours(esiklenmis,cv2.RETR_LIST,cv2.CHAIN_APPROX_NONE)
        cv2.drawContours(img2,sinirlar,-1,(255,0,0),4)
        cnt = sinirlar[0]
        M = cv2.moments(cnt)
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
        print("Nesne X ekseni = ",cx," /  Nesne Y ekseni = ",cy,"  / Nesne Id = ", self.nesne_id  )

Kamera()




        
        
        
        
