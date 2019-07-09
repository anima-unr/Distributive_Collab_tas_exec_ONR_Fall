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
import math
count = 0
 
#fgbg = cv2.BackgroundSubtractorMOG(500,5,.7,15)

fgbg = cv2.BackgroundSubtractorMOG()
first_gray = np.zeros((500,500,3), np.uint8)

def unit_vector(vector):
  return vector / np.linalg.norm(vector)

def angle_between(v1, v2):
    # """ Returns the angle in radians between vectors 'v1' and 'v2'::

    #         >>> angle_between((1, 0, 0), (0, 1, 0))
    #         1.5707963267948966
    #         >>> angle_between((1, 0, 0), (1, 0, 0))
    #         0.0
    #         >>> angle_between((1, 0, 0), (-1, 0, 0))
    #         3.141592653589793"""
  v1_u = unit_vector(v1)
  v2_u = unit_vector(v2)
  return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))

def calculating_vectors(start_x,start_y,end_x,end_y,far_x,far_y):   #calculating vectors from given points 
  global v1,v2
  v1 = []
  v2 = []
  
  v1.append(start_x - far_x)  #vector between  hand -> obj1
  v1.append(start_y - far_y)

  v2.append(end_x - far_x)  #vector between  hand -> obj1
  v2.append(end_y - far_y) 

class image_converter:




  def __init__(self):
    self.image_pub = rospy.Publisher("image_topic_2",Image)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/kinect2/qhd/image_color",Image,self.callback)

  def callback(self,data):

    global count,fgmask,first_gray,v1,v2
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    height , width , layers =  cv_image.shape
    new_h=height/2
    new_w=width/2
    


    #fgmask = cv2.resize(fgmask, (new_w, new_h))
    frame = cv2.resize(cv_image, (480, 270))
    x = 40
    y = 60

    cv2.rectangle(frame, (x,y), (x+new_w-80, y+new_h-80), (0, 255, 0), 2)
    roi = frame[y:y+new_h-80, x:x+new_w-80]

    #cv2.circle(frame,(480,80),10,[0,0,255],-1)
    cv2.imshow("Hand Postion - 5th Stepppppppppppppp", frame)
    cv2.imshow("Hand Postion - 5th Stepppppppppppppp", roi)
    #first_image = cv2.imread('first_frame.jpg')
    if count==0:
      first_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

      first_gray = cv2.GaussianBlur(first_gray, (21, 21), 0)
      count=1


    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (21, 21), 0)

    difference = cv2.absdiff(gray, first_gray)

    thresh = cv2.threshold(difference, 25, 255, cv2.THRESH_BINARY)[1]
    thresh = cv2.dilate(thresh, None, iterations=2)

    cv2.imshow("Resizing Image-1st Step", frame)
   
    cv2.imshow("Background Subtraction-2nd Step", thresh)

   

    lower = np.array([0, 48, 80], dtype = "uint8")
    upper = np.array([20, 255, 255], dtype = "uint8")

    converted = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
    skinMask = cv2.inRange(converted, lower, upper)

    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (1, 1))
    skinMask = cv2.erode(skinMask, kernel, iterations = 2)
    skinMask = cv2.dilate(skinMask, kernel, iterations = 2)

    skin = cv2.bitwise_and(gray, gray, mask = skinMask)

    cv2.imshow("Skin Detection-3rd Step", skin)

    skin = cv2.bitwise_and(skin, skin, mask = thresh)
    thresh2 = cv2.threshold(skin, 25, 255, cv2.THRESH_BINARY)[1]

    cv2.imshow("Segmenting the Hand-4th Step", thresh2)


    contours,hierarchy = cv2.findContours(thresh2,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(thresh2, contours, 0, (0, 255, 0), 3)

    cv2.imshow("Drawing Contours - 5th Step", thresh2)


    #im2,contours,hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    if len(contours) != 0:
        max_angle = 0
    # draw in blue the contours that were founded
        cv2.drawContours(thresh2, contours, -1, 255, 3)

    #find the biggest area
        c = max(contours, key = cv2.contourArea)

        x,y,w,h = cv2.boundingRect(c)
    # draw the book contour (in green)
        #if(y>80):
        cv2.rectangle(frame,(x,y),(x+w,y+h),(0,255,0),2)
        cv2.imshow("Hand Postion - 5th Step", frame)

        if(y>80):


          hull = cv2.convexHull(c,returnPoints = False)
          defects = cv2.convexityDefects(c,hull)

          for i in range(defects.shape[0]):
            s,e,f,d = defects[i,0]
            start = tuple(c[s][0])
            end = tuple(c[e][0])
            far = tuple(c[f][0])
            cv2.line(frame,start,end,[0,255,0],2)
            cv2.circle(frame,far,5,[0,0,255],-1)
            
            calculating_vectors(start[0],start[1],end[0],end[1],far[0],far[1])
            ang = angle_between(v1,v2)
            if(ang>max_angle):
              max_angle = (int)(math.degrees(ang))
          print(max_angle)
          with open("eang.txt", "a") as f:
            f.write("\n %d checkkkk"%(max_angle))

        cv2.imshow('img',frame)


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
