#!/usr/bin/env python
from __future__ import print_function


import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from pyimagesearch import imutils
import numpy as np
import argparse
count = 0
 
#fgbg = cv2.BackgroundSubtractorMOG(500,5,.7,15)
fgbg = cv2.BackgroundSubtractorMOG()
#fgmask = np.zeros((500,500,3), np.uint8)
class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("image_topic_2",Image)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/kinect2/qhd/image_color",Image,self.callback)

  def callback(self,data):
    global count,fgmask
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    #first_image = cv2.imread('first_frame.jpg')
    
    


    

    lower = np.array([0, 48, 80], dtype = "uint8")
    upper = np.array([20, 255, 255], dtype = "uint8")

    height , width , layers =  cv_image.shape
    new_h=height/2
    new_w=width/2



    #fgmask = cv2.resize(fgmask, (new_w, new_h))
    frame = cv2.resize(cv_image, (new_w, new_h))
    converted = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
    skinMask = cv2.inRange(converted, lower, upper)
    
    cv2.imshow("first_step", frame)
    

   
    
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (1, 1))
    skinMask = cv2.erode(skinMask, kernel, iterations = 2)
    skinMask = cv2.dilate(skinMask, kernel, iterations = 2)
    
  # blur the mask to help remove noise, then apply the
  # mask to the frame
    skinMask = cv2.GaussianBlur(skinMask, (3, 3), 0)
    cv2.imshow("second_step", skinMask)


        #else: 
          #skinMask[i,j] = 255
          
    #cv2.imshow("background_extract", skinMask)     
    #skin = cv2.bitwise_and(frame, frame, mask = skinMask)
 
  # show the skin in the image along with the mask

    '''if count == 0:
      fgmask = fgbg.apply(skinMask)
      print("helooooooooooooooooooooo")
      count = 1'''
    fgmask = fgbg.apply(skinMask)
    skin = cv2.bitwise_and(fgmask, fgmask, mask = skinMask)
    #fgmask = cv2.resize(fgmask, (new_w, new_h))
    #cv2.imshow("skinMask", skinMask)
    cv2.imshow("third_step_BG_extract", fgmask)
    cv2.imshow("Image window_final", skin)

    image3 = fgmask - skin 
    
    ret,thresh = cv2.threshold(skin,127,255,0)
    
    contours,hierarchy = cv2.findContours(thresh,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    
    #cnts = cv2.findContours(skin.copy(), cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
    
    

    #cv2.drawContours(thresh, contours, 0, (0, 255, 0), 3)
    #cnts = sorted(contours, key = cv2.contourArea, reverse = True)[:20]
    '''screenCnt=None
    for c in cnts:
      screenCnt=c'''
    largest_area = 0
    largest_area_index = 0
    i = 0
    for c in contours:
      a = cv2.contourArea(c) 
      if a>largest_area:
        largest_area = a
        largest_area_index = i
        i=i+1

    #cnt = cnts[1]
    print(largest_area)
    print(largest_area_index)
    cv2.drawContours(thresh, contours, largest_area_index, (0, 255, 0), 3)
    cv2.imshow("Image window_final2", thresh)
    
    
    i = 0
    for c in contours:
      if i==largest_area_index:
        
        #(x, y, w, h) = cv2.boundingRect(c)
        #print("**********************************************************%d %d %d %d"%(x,y,w,h))
        #cv2.rectangle(skin, (x,y), (x+w,y+h), (0,255,0), 2)
        (x,y),radius = cv2.minEnclosingCircle(c)
        center = (int(x),int(y))
        radius = 10
        print("**********************************************************%d %d %d"%(x,y,radius))
        cv2.circle(frame,center,radius,(0,255,0),2)
      i=i+1
    #cv2.imshow("Image window_final22", frame)

   
# need an extra "min/max" for contours outside the frame


    

    '''for i in range(0,len(cnts)):
      cv2.drawContours(skin, [cnts], i, (0, 255, 0), 3)'''
    #cv2.imshow("Image window", np.hstack([skin,frame]))
    cv2.waitKey(3)

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(converted, "bgr8"))
    except CvBridgeError as e:
      print(e)

def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
