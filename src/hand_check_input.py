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
from detecting_hand.srv import Conv2DTo3D
from detecting_hand.msg import bounding_box_calculated_center
from robotics_task_tree_msgs.msg import ObjStatus
from keras.models import load_model
count = 0
p = 3695
#fgbg = cv2.BackgroundSubtractorMOG(500,5,.7,15)
model = load_model('/home/bashira/ML_virtual/saved_model_1.hdf5')
fgbg = cv2.BackgroundSubtractorMOG()
first_gray = np.zeros((500,500,3), np.uint8)

hand_status_pub = rospy.Publisher('/hand_bounding_box_center', bounding_box_calculated_center, queue_size=10)

def hand_service(x,y):
        rospy.wait_for_service('/conv_coord') 
        try:
            print("helloo")
            conv_coord = rospy.ServiceProxy('/conv_coord', Conv2DTo3D)
            msg = conv_coord(x,y)
        except rospy.ServiceException, e:
            print("Service call failed: %s"%e)
    


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

def calculateFingers(res, drawing,x,w,y,h):
    #  convexity defect
    hull = cv2.convexHull(res, returnPoints=False)
    if len(hull) > 2:
        defects = cv2.convexityDefects(res, hull)
        if defects is not None:
            cnt = 0
            for i in range(defects.shape[0]):  # calculate the angle
                s, e, f, d = defects[i][0]
                start = tuple(res[s][0])
                end = tuple(res[e][0])
                far = tuple(res[f][0])
                a = math.sqrt((end[0] - start[0]) ** 2 + (end[1] - start[1]) ** 2)
                b = math.sqrt((far[0] - start[0]) ** 2 + (far[1] - start[1]) ** 2)
                c = math.sqrt((end[0] - far[0]) ** 2 + (end[1] - far[1]) ** 2)
                angle = math.acos((b ** 2 + c ** 2 - a ** 2) / (2 * b * c))  # cosine theorem
                font = cv2.FONT_HERSHEY_SIMPLEX
                


                if angle <= math.pi / 2:  # angle less than 90 degree, treat as fingers
                    print('%d@@@@@@'%(math.degrees(angle)))
                    if(math.degrees(angle)<45):
                        print('small')
                        cv2.putText(drawing,'Small!',(x+w,y+h), font, 0.5, (200,255,155), 2)
                    else:
                        print('large')
                        cv2.putText(drawing,'Large!',(x+w,y+h), font, 0.5, (200,255,155), 2)


                    cnt += 1
                    cv2.circle(drawing, far, 3, [211, 84, 0], -1)
                    cv2.circle(drawing, start, 3, [211, 84, 0], -1)
                    cv2.circle(drawing, end, 3, [211, 84, 0], -1)
            if cnt > 0:
              return True, cnt+1
            else:
              return True, 0
    return False, 0

class image_converter:




  def __init__(self):
    self.image_pub = rospy.Publisher("image_topic_2",Image)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/kinect2/qhd/image_color",Image,self.callback)

  def callback(self,data):

    global count,fgmask,first_gray,v1,v2,p
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
    y = 40

    cv2.rectangle(frame, (x,y), (x+new_w-80, y+new_h-80), (0, 255, 0), 2)
    roi = frame[y:y+new_h-80, x:x+new_w-80]
    print(roi.shape)
    #cv2.circle(roi,(100,10),15,[0,0,255],-1)
    cv2.imshow("check", roi)

    #cv2.circle(frame,(480,80),10,[0,0,255],-1)
    #cv2.imshow("Hand Postion - 5th Stepppppppppppppp", frame)
    #cv2.imshow("Hand Postion - 5th Stepppppppppppppp", roi)
    #first_image = cv2.imread('first_frame.jpg')
    if count==0:
      first_gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)

      first_gray = cv2.GaussianBlur(first_gray, (21, 21), 0)
      count=1


    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    gray_ = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (21, 21), 0)
    gray_thresh = cv2.threshold(gray, 25, 255, cv2.THRESH_BINARY)[1]

    difference = cv2.absdiff(gray, first_gray)
    #cv2.imshow("Hand Postion - 5th Stepp", difference)
    #first_image = cv2.imread('first_frame.jpg')
    thresh = cv2.threshold(difference, 25, 255, cv2.THRESH_BINARY)[1]
    thresh = cv2.dilate(thresh, None, iterations=2)

    cv2.imshow("Resizing Image-1st Step", frame)
   
    cv2.imshow("Background Subtraction-2nd Step", thresh)

   

    lower = np.array([0, 48, 80], dtype = "uint8")
    upper = np.array([20, 255, 255], dtype = "uint8")

    converted = cv2.cvtColor(roi,cv2.COLOR_BGR2HSV)
    skinMask = cv2.inRange(converted, lower, upper)

    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (1, 1))
    skinMask = cv2.erode(skinMask, kernel, iterations = 2)
    skinMask = cv2.dilate(skinMask, kernel, iterations = 2)

    skinMask = cv2.GaussianBlur(skinMask, (3, 3), 0)

    skin = cv2.bitwise_and(gray_thresh, gray_thresh, mask = skinMask)

    cv2.imshow("Skin Detection-3rd Step", skin)

    skin = cv2.bitwise_and(skin, skin, mask = thresh)
    thresh2 = cv2.threshold(skin, 25, 255, cv2.THRESH_BINARY)[1]

    cv2.imshow("Segmenting the Hand-4th Step", thresh2)


    contours,hierarchy = cv2.findContours(thresh2,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(thresh2, contours, 0, (0, 255, 0), 3)


    cv2.imshow("Drawing Contours - 5th Step", thresh2)


    # res = cv2.resize(thresh2,None,fx=2, fy=2, interpolation = cv2.INTER_CUBIC)
    # cv2.imshow("res", res)

    #im2,contours,hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    if len(contours) != 0:
        max_angle = 0
        maxArea = -1
    # draw in blue the contours that were founded
        #cv2.drawContours(thresh2, contours, -1, 255, 3)
        for i in xrange(len(contours)):
            temp = contours[i]
            area = cv2.contourArea(temp)
            if area > maxArea:
                maxArea = area
                ci = i

        c = contours[ci]

    #find the biggest area
        #c = max(contours, key = cv2.contourArea)
        
        x,y,w,h = cv2.boundingRect(c)
        # gray2 = gray_[y:y+h, x:x+w]
        # cv2.imshow("check box", gray2)
        # cv2.imwrite("image_gray/large {0}.jpg".format(i),gray2)
        # i = i+1

        print("%d %d %d %d"%(x,y,w,h))
        cent_x = x + (w/2)
        cent_y = y + (h/2)
        rospy.wait_for_service('/conv_coord') 
        try:
            print("helloo")
            conv_coord = rospy.ServiceProxy('/conv_coord', Conv2DTo3D)
            msg = conv_coord(x,y)
        except rospy.ServiceException, e:
            print("Service call failed: %s"%e)

        #hand_service(cent_x,cent_y)


        print("%f %f %f"%(msg.newZ,msg.newX,msg.newY))
        hand_msg = bounding_box_calculated_center()
        hand_msg.x = msg.newX
        hand_msg.y = msg.newY
        hand_msg.z = msg.newZ
        hand_msg.x2d = cent_x
        hand_msg.y2d = cent_y

        hand_status_pub.publish(hand_msg)
    # draw the book contour (in green)
        #if(y>80):

        cv2.circle(roi,(cent_x,cent_y),5,[0,0,255],-1)
        cv2.rectangle(roi,(x,y),(x+w,y+h),(0,255,0),2)

        res = cv2.resize(roi,None,fx=2, fy=2, interpolation = cv2.INTER_CUBIC)
        
        cv2.imshow("Hand Postion - 6th Step", res)

        hull = cv2.convexHull(c)
        drawing = np.zeros(roi.shape, np.uint8)
        drawing2 = np.zeros(roi.shape, np.uint8)
        cv2.drawContours(drawing, [c], 0, (0, 255, 0), 2)
        cv2.drawContours(drawing2, [c], 0, (255, 255, 255), -1)
        x,y,w,h = cv2.boundingRect(c)
        drawMask = cv2.erode(drawing2, kernel, iterations = 2)
        drawMask = cv2.dilate(drawMask, kernel, iterations = 2)

        #drawMask = cv2.GaussianBlur(drawMask, (3, 3), 0)
        drawing3 = drawMask[y-10:y+h+10, x-10:x+w+10]
        cv2.imshow("check box", drawing3)
        #cv2.imwrite("image/small_test {0}.jpg".format(p),drawing3)
        p = p+1
        print('%d'%(p))
        cv2.drawContours(drawing, [hull], 0, (0, 0, 255), 3)

        cv2.imshow("hello", drawing2)

        isFinishCal, cnt = calculateFingers(c, drawing,x,w,y,h)
        print("Fingers", cnt)

        res = cv2.resize(drawing,None,fx=2, fy=2, interpolation = cv2.INTER_CUBIC)
        
        cv2.imshow("Finger Postion - 7th Step", res)
        #cv2.imshow('Finger Postion - 6th Step', drawing)

        #if(y>80):


        # hull = cv2.convexHull(c,returnPoints = False)
        # defects = cv2.convexityDefects(c,hull)

        # for i in range(defects.shape[0]):
        #   s,e,f,d = defects[i,0]
        #   start = tuple(c[s][0])
        #   end = tuple(c[e][0])
        #   far = tuple(c[f][0])
        #   cv2.line(roi,start,end,[0,255,0],2)
        #   cv2.circle(roi,far,5,[0,0,255],-1)
            
        #   calculating_vectors(start[0],start[1],end[0],end[1],far[0],far[1])
        #   ang = angle_between(v1,v2)
        #   if(ang>max_angle):
        #     max_angle = (int)(math.degrees(ang))
        # print(max_angle)
        # with open("eang.txt", "a") as f:
        #   f.write("\n %d checkkkk"%(max_angle))

        # cv2.imshow('img',roi)


    cv2.waitKey(3)
    
    
    

    # try:
    #   self.image_pub.publish(self.bridge.cv2_to_imgmsg(converted, "bgr8"))
    # except CvBridgeError as e:
    #   print(e)

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
