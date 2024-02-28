#!/usr/bin/env python3
#_*_ coding:utf-8 _*_
import rospy
import cv2
from cv_bridge import CvBridge ,CvBridgeError
import numpy as np
from sensor_msgs.msg import CompressedImage
from dh_pkg.msg import cam_msg

class IMGParser:
    def __init__(self):
        rospy.init_node('image_parser', anonymous=True)
        self.image_sub = rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.callback)
        self.cam_pub = rospy.Publisher('/Cam_dectection', cam_msg, queue_size =10)
        self.cam_detect_msg = cam_msg() 
        self.image_msg = CompressedImage()
        self.bridge = CvBridge()
        self.blueMin = np.array([110,229,0])  #blue hsv 범위 최소값
        self.blueMax = np.array([117,253,54]) #blue hsv 범위 최대값
        # self.crops = np.array([[[40,265],
        #                        [40,230],
        #                        [320,230],
        #                        [320,265]]])  #roi 범위 지정
        self.crops = np.array([[[20,265],
                               [20,230],
                               [340,230],
                               [340,265]]])  #roi 범위 지정
        self.blue_img_roi =[]
        self.is_dynamic_flag=False

                
    def colorFilter(self, img) :
        img_c = img.copy()
        img_hsv= cv2.cvtColor(img_c,cv2.COLOR_BGR2HSV)
        b = cv2.inRange(img_hsv,self.blueMin,self.blueMax)
        
        return b

    def selectRoi(self, arr) :
        arr2 = arr.copy()
        mask = np.zeros_like(arr2, np.uint8)
        if (len(arr2.shape)==3) :
            mask_value = (255,255,255)
        else :
            mask_value = (255)
        cv2.fillPoly(mask,self.crops, mask_value)
        mask = cv2.bitwise_and(mask,arr2)
        return mask
      
    def callback(self, msg):
        try:
            self.image_msg = msg
            cv_img = self.bridge.compressed_imgmsg_to_cv2(self.image_msg)
            
        except CvBridgeError as e:
            print(e)

        b_img =self.colorFilter(cv_img) 
        b_img =self.selectRoi(b_img)
        self.blue_img_roi = b_img

       
        bin_img = np.zeros_like(self.blue_img_roi)
        
        bin_img[self.blue_img_roi>10] = 1
        by = np.sum(bin_img, axis=0)

        if(len(by[by>3]) >3 ) : #값이 많이 튀면 임계값 증가시켜보자
            self.is_dynamic_flag = True
        else :
            if(self.is_dynamic_flag==True) :
                if(len(by[by>3])<1) : 
                    self.is_dynamic_flag=False
    
        self.cam_detect_msg.is_dynamic_obs = self.is_dynamic_flag
        self.cam_pub.publish(self.cam_detect_msg)
        # cv2.imshow("img",cc)
        # cv2.imshow("img2",self.blue_img_roi)

        # cv2.waitKey(1)
        

def main() :
    try :
        image_parser = IMGParser()
        rospy.spin()
    except rospy.ROSInitException :
        pass


if __name__ == '__main__':
    main()