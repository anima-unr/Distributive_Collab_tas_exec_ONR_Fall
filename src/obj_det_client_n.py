#!/usr/bin/env python
import rospy
import sys
import time
import math
import os
import numpy as np
import cv2
#from typing import NamedTuple

from gpd.msg import GraspConfigList
from darknet_ros_msgs.msg import BoundingBoxes
from darknet_ros_msgs.msg import BoundingBox
from std_msgs.msg import String
from sensor_msgs.msg import Image
from detecting_hand.srv import Vision_Service #importing srv msg from vision code
from detecting_hand.srv import Conv2DTo3D
from detecting_hand.msg import bounding_box_calculated_center
from detecting_hand.msg import msg2dto3d
from detecting_hand.msg import msg2dto3d_object
from detecting_hand.msg import Vision_Message
from cv_bridge import CvBridge, CvBridgeError
from robotics_task_tree_msgs.msg import ObjStatus
from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import *

def processFeedback(feedback):
    p = feedback.pose.position
    print feedback.marker_name + " is now at " + str(p.x) + ", " + str(p.y) + ", " + str(p.z)

theta1_obj = [0,0,0,0,0,0,0]
theta2_obj = [0,0,0,0,0,0,0]
prev_theta1_obj = [0,0,0,0,0,0,0]
prev_theta2_obj = [0,0,0,0,0,0,0]

hand_change_pos = 0

unit_v = [0,0,0,0,0,0,0]
unit_v_ = [-1,-1,-1,-1,-1,-1,-1]

count = 0
val_1 = 0
val_4 = 0
val_3 = 0
val_2 = 0
val_5 = 0
flag = 1
hand_x_sum = 0
hand_x_avg = 0
hand_y_avg =0
hand_y_sum = 0
hand_z_sum = 0
hand_z_avg = 0
hand_count = 0
change = 2
change2 = 2

prev_hand_x_sum=0
prev_hand_y_sum=0

hand_loc = []
hand_first_position = 0
unit_v1 = []
unit_v11 =[]
v1 = []
v11 = []
v3 = []
v4 = []
count_inside = 0;
count_cup=0;
count_scissor=0;


draw_box = 0


c1=0
c2=0
c3=0
c4=0
c5=0
c6=0
c7=0

obj_chance = [0,0,0,0,0,0,0]
obj_chance2 = [0,0,0,0,0,0,0]
obj_c = [ 0,0,0]
v_obj = [0,0,0,0,0,0,0]


val_obj = [0,0,0]
object_list = ["Tea_Pot","Apple","Cup","Burger","Sugar","Sandwich","Orange"]

distance = [0,0,0,0,0]

class location:
    center_x_1 = 0
    center_y_1 = 0
    center_x_2 = 0
    center_y_2 = 0

class location_center_3d:
    center_x = 0
    center_y = 0

class location_center_hand:
    center_x = 0
    center_y = 0
    center_z = 0
    center_x_2d = 0
    center_y_2d = 0
class location_center:
    center_x = 0
    center_y = 0
    center_z = 0
   

hand = location_center_hand() 
hand_ = location_center()
obj1 = location_center() #first point of the cup
obj2 = location_center() #second point of the cup
obj1_ = []
obj2_ = []


cup_fix_loc = location_center()

fix_obj_flag = [0,0,0,0,0,0,0]



hand_fix_flag = 0
hand_change = 0

obj_check = [1,1,1,1,1,1,1]



object_started_flag = 0
started = [0,0,0,0,0,0,0] #initially all the started flag for the objects are zero 
done = [0,0,0,0,0,0,0] #initially all the done flag for the objects are zero 

hand_i = 0
hand_i_x = 0
hand_i_y = 0

flag_sv = 0
l = 3
cup_status_pub = rospy.Publisher('/Cup_status', ObjStatus, queue_size=10)
tea_pot_status_pub = rospy.Publisher('/Tea_Pot_status', ObjStatus, queue_size=10)
sugar_status_pub = rospy.Publisher('/Sugar_status', ObjStatus, queue_size=10)
burger_status_pub = rospy.Publisher('/Burger_status', ObjStatus, queue_size=10)
orange_status_pub = rospy.Publisher('/orange_status', ObjStatus, queue_size=10)
sandwich_status_pub = rospy.Publisher('/sandwich_status', ObjStatus, queue_size=10)
apple_status_pub = rospy.Publisher('/Apple_status', ObjStatus, queue_size=10)


def distance_object(x2,y2,x1,y1):
	return math.sqrt((y2-y1)*(y2-y1) + (x2-x1)*(x2-x1))


def center_callback(hand_center): #storing the hand location 
	
     global hand,flag,hand_i,hand_i_x,hand_i_y,flag_sv,fix_obj_flag
     hand.center_x=hand_center.x
     hand.center_y=hand_center.y
     hand.center_z=hand_center.z
     hand.center_x_2d=hand_center.x2d
     hand.center_y_2d=hand_center.y2d
     #hand.center_z=hand_center.z
     if hand_i == 0:		#storing the first location after running the program
		hand_i_x = hand_center.x2d
		hand_i_y = hand_center.y2d
		hand_i = 1
     if flag_sv == 0:
     	get_obj_loc()
	if fix_obj_flag.count(1) == l:
		flag_sv =1
		print "cjecl"
     callback()




#
def get_obj_loc():
	global obj1_,obj2_

	obj1 = location_center() #first point of the cup  NewX: -0.142711 NewY: 0.085340 NewZ: 0.753000



	obj1.center_x = -0.0867409557104
	obj1.center_y = 0.1251219064 
	obj1.center_z = 0.862000048161

				
	obj1_.append(obj1)




	obj1 = location_center() #first point of the cup NewX: 0.000828 NewY: -0.073658 NewZ: 0.869000

	obj1.center_x = 0.300140023232
	obj1.center_y = -0.0920200049877
	obj1.center_z =  0.903000056744

				
	obj1_.append(obj1)


	obj1 = location_center() #first point of the cup NewX: 0.255453 NewY: 0.090116 NewZ: 0.782000


	obj1.center_x = 0.188020005822
	obj1.center_y = 0.0579133369029
	obj1.center_z = 0.833000063896

				
	obj1_.append(obj1)

	obj1 = location_center() #first point of the cup NewX: 0.492566 NewY: 0.056585 NewZ: 0.974000

	obj1.center_x = 0.0467942878604 
	obj1.center_y = -0.104260958731
	obj1.center_z = 0.862000048161

				
	obj1_.append(obj1)




	fix_obj_flag = [1,1,1,1]



	
	  

 
     
    
def get_obj_loc_client(): #service call to find the object location from YOLO


    rospy.wait_for_service('/CV_Objects') #service name
    #print "hello"
    try:
        CV_Objects = rospy.ServiceProxy('/CV_Objects', Vision_Service) #service name 
	
	global hand_loc,hand_count,hand_fix_flag,val_1,val_2,val_3,val_4,val_5
	global fix_obj_flag,object_list,obj_check,hand,obj1,obj2,obj1_,obj2_  
	
	#if 105 < hand.center_x <110 and 237 < hand.center_y < 69 == 242:
	'''msg = CV_objects("hand")
	if msg.object_location[0].Found == 1:
		hand.center_x = msg.object_location[0].Pos.x
		hand.center_y = msg.object_location[0].Pos.y'''
	
	'''if hand_i_x-20<hand.center_x<hand_i_x+20 and  hand_i_y-20 <=hand.center_y<=hand_i_y+20:
		print" "
	else:	'''	
	for x in range(0, l):
		if obj_check[x] == 1:
			msg = CV_Objects(object_list[x])
			if msg and fix_obj_flag[x]==0:
				obj1 = location_center() #first point of the cup
				#obj2 = location_center() #second point of the cup

				obj1.center_x = msg.object_location[0].Pos.x 
				obj1.center_y = msg.object_location[0].Pos.y
				obj1.center_z = msg.object_location[0].Pos.z
				'''obj2.center_x = msg.object_location[0].Pos.x + .0205
				obj2.center_y = msg.object_location[0].Pos.y
				obj2.center_z = msg.object_location[0].Pos.z'''
				
				obj1_.append(obj1)
				#obj2_.append(obj2)
				#print "~~~~%f %f %f\n"%(obj2_[x].center_x,obj2_[x].center_y,obj2_[x].center_z)

				fix_obj_flag[x] = 1
				
				#obj_loc_save
			elif not msg:
				fix_obj_flag[x]=2
				obj_check[x]=0
				hand_count=0
				hand_fix_flag=2
				hand_loc = []
				#print "Hi"

	
		

	

		
	 
 
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
    
# def unit_vector(vector):
# 	return vector / np.linalg.norm(vector)
# def unit_vector_direction(vector): #calculating angle between two vectors 
#     """ Returns the unit vector of the vector.  """
#     vector = vector / np.linalg.norm(vector)
#     return math.degrees(math.atan2(vector[1],vector[0]))


def angle_between(v1, v2, acute):
    with open("example2.txt", "a") as f:
	f.write("%s %s\n"%(v1,v2))
    """ Returns the angle in radians between vectors 'v1' and 'v2'::

            >>> angle_between((1, 0, 0), (0, 1, 0))
            1.5707963267948966
            >>> angle_between((1, 0, 0), (1, 0, 0))
            0.0
            >>> angle_between((1, 0, 0), (-1, 0, 0))
            3.141592653589793
    """
     
    '''v1_u = unit_vector(v1)
    v2_u = unit_vector(v2)
    return round(np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0)),2)'''
    #return np.arccos(np.dot(v1,v2) / len(v1) * len(v2))
    angle = np.arccos(np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2)))
    if (acute == True):
        return angle
    else:
        return 2 * np.pi - angle

def unit_vector(vector):
    return vector / np.linalg.norm(vector)  

def calculating_vectors(objx1,objy1,objz1,handx,handy,handz,handxx,handyy,handzz):   #calculating vectors from given points 
	global v1,v11
	v1 = []
	v11 =  []
	
	v1.append(objx1 - handx)  #vector between  hand -> obj1
	v1.append(objy1 - handy)
	v1.append(objz1 - handz)

	v11.append(objx1 - handxx)  #vector between  handxx -> obj1
	v11.append(objy1 - handyy)
	v11.append(objz1 - handzz)
	

	'''v2.append(objx2 - handx) #vector between  hand ->obj
	v2.append(objy2 - handy)
	#v2.append(handz-objz1)
	

	v3.append(objx1-objx2) #vector between obj2 -> obj1
	v3.append(objy1-objy2)
	#v3.append(objz1-objz2)
	

	#v4.append(handx-objx2) #vector between obj2 -> hand
	#v4.append(handy-objy2)
	#v4.append(handz-objz2)'''

def cosine_similarity(v1,v2):
    "compute cosine similarity of v1 to v2: (v1 dot v2)/{||v1||*||v2||)"
    with open("example2.txt", "a") as f:
		f.write("%s %s %d\n"%(v1,v2,count))
    sumxx, sumxy, sumyy = 0, 0, 0
    for i in range(len(v1)):
        x = v1[i]; y = v2[i]
        sumxx += x*x
        sumyy += y*y
        sumxy += x*y
    return (sumxy/math.sqrt(sumxx*sumyy))*100
def intention_object(theta,unit_vector_):  #comparing the angles which are created with the two points of the object and the hand
 	'''if theta1==prev_theta1 and theta2==prev_theta2:       #if both the angle decrease from the previous angles then the hand is moving towards the object. 

		val = 1
	elif theta1<prev_theta1 and theta2<prev_theta2:
		val = 2
	elif theta1<prev_theta1 and theta2==prev_theta2:
		val = 2
	elif theta1<=prev_theta1 and theta2<prev_theta2:
		val = 2
	elif theta1==prev_theta1 and theta2<prev_theta2:
		val = 2
	else:
		val = 0'''
	if theta <= unit_vector_ -10  and theta >= unit_vector_ + 10:

		val = 1

	'''if theta1==prev_theta1 and theta2==prev_theta2:       #if both the angle decrease from the previous angles then the hand is moving towards the object. 

		val = 1
	elif theta1<prev_theta1 and theta2>prev_theta2:
		val = 2

	elif theta1<prev_theta1 and theta2==prev_theta2:
		val = 2
	elif theta1==prev_theta1 and theta2>prev_theta2:
		val = 2'''
	#else:
	#	val = 0
	#elif theta1==prev_theta1 and theta2<prev_theta2:
		#val = 2

	
	return val	

def calculating_intention(prev_val_obj,val_,i):
	global val_obj 
	if prev_val_obj[i] == 0 and val_ == 1:
		val_obj[i] = 1
	elif prev_val_obj[i] == 1 and val_ == 1:
		val_obj[i] = 2

def c(x):
	global c1,c2,c3,c4,c5,c6,c7
	if x == 0:
		c1 = c1+1
	elif x == 1:
		c2= c2+1
	elif x ==2:
		c3 = c3+1
	elif x == 3:
		c4= c4+1
	elif x ==4:
		c5 = c5+1
	elif x == 5:
		c6= c6+1
	elif x ==6:
		c7 = c7+1

def callback():
	global count,hand_count,hand_x_sum,hand_y_sum,hand_z_sum,hand_loc,prev_hand_x_avg,prev_hand_y_avg,prev_hand_z_avg,hand_x_avg,hand_y_avg,hand_z_avg,hand,obj1,obj2,v1,v2,v3,v4,flag,obj_fix1,obj_fix2
	#time.sleep(0.01)
	global change,change2,count_inside,draw_box,object_started_flag	
	global hand_loc_save,hand_fix_flag,hand_save_x,hand_save_y,distance,hand_i_x,hand_i_y,hand_first_position,hand_ 
	global obj_chance,v_obj,val_obj,obj2_,obj1_,c1,c2,c3,c4,c5,c6,c7,unit_v_,unit_v,obj_chance2,hand,hand_change_pos,obj_c

	
	#if hand_i_x-20<hand.center_x_2d<hand_i_x+20 and  hand_i_y-20 <=hand.center_y_2d<=hand_i_y+20: #it compares the hand position with the initial position that is stored in the center_callback function.
	#if hand.center_x == hand_i_x and hand.center_y == hand_i_y:
	#if hand.center_y_2d <= 10:
	if hand.center_y_2d <47:
					                #After each turn, in this part the chance value and started value are assigned initial value and the hand goes for another object.                   
		print "loc no change~~~~~~~~~~~~"
		print "%f %f %d %d\n"%(hand_i_x,hand_i_y,hand.center_x_2d,hand.center_y_2d)

		obj_chance2 = [0,0,0,0,0,0,0]
		obj_c = [ 0,0,0,0]
		c1=0
		c2=0
		c3=0
		c4=0
		c5=0 
		c6=0
		c7=0
		object_started_flag = 0
		hand_change_pos = 0
		hand_first_position = 0
		hand_count = 0
		for x in range (0,l):    #it will check if any started flag is one. if it is then it will make the done flag value to one. 
			if started[x] == 1:
				done[x]=1
				started[x]=0	
	elif hand.center_y_2d > 47 and hand.center_y_2d < 57:
		hand_change_pos = 1
			
	if hand_change_pos == 1:
		
		print "loc  change~~~@@@@@@@@@@@~~~~~~~"	

		
		hand_loc.append(hand) #storing the hand location in a list for average
		with open("example_check_.txt", "a") as f:
			f.write("\n %d helloooooo %d %f"%(count,hand_first_position,hand_.center_x))	

		# hand_x_sum=hand_x_sum + hand_loc[hand_count].center_x;
		# hand_y_sum=hand_y_sum + hand_loc[hand_count].center_y;
		# hand_z_sum=hand_y_sum + hand_loc[hand_count].center_z;
		hand_x_sum = hand.center_x
		hand_y_sum = hand.center_y
		hand_z_sum = hand.center_z


		
		if hand_first_position == 0:
			hand_first_position = 10
			hand_.center_x = hand_x_sum
			hand_.center_y = hand_y_sum
			hand_.center_z = hand_z_sum
	    	#with open("example_check_.txt", "a") as f:
			#	f.write("\n%d hello %d %f %f"%(count,hand_first_position,hand_x_sum,hand_y_sum))		 
	
		
		hand_count = hand_count + 1;

			
		if count % 1 == 0:   #average the hand values after 2 rounds
			count_inside=count_inside+1
			prev_hand_x_avg = hand_x_avg
			prev_hand_y_avg = hand_y_avg
			prev_hand_z_avg = hand_z_avg
			hand_x_avg = hand_x_sum/1
			hand_y_avg = hand_y_sum/1
			hand_z_avg = hand_z_sum/1
		
			prev_val_obj= val_obj 

			for x in range(0, l):
				if done[x] == 1 and v_obj[x]!=-1:
					obj_chance2 = [0,0,0,0,0,0,0]
					c1=0
					c2=0
					c3=0
					c4=0
					c5=0
					c6=0
					c7=0
					v_obj[x] = -1
					hand_fix_flag = 2
					obj_c = [ 0,0,0,0]
					#object_started_flag = 0
				if done[x] == 1:
					val_obj[x] = -2 

				else:
					#calculating_vectors(obj2_[x].center_x,obj2_[x].center_y,obj2_[x].center_z,obj1_[x].center_x,obj1_[x].center_y,obj1_[x].center_z,hand_x_avg,hand_y_avg,hand_z_avg)
					#print "%s"%(object_list)
					with open("exampleval.txt", "a") as f:
						f.write("\nvalue %d %s %d %f"%(count,object_list[x],x,obj1_[x].center_x))
					calculating_vectors(obj1_[x].center_x,obj1_[x].center_y,obj1_[x].center_z,hand_x_avg,hand_y_avg,hand_z_avg,hand_.center_x,hand_.center_y,hand_.center_z)
					
					val_obj[x] = (int)(cosine_similarity(unit_vector(v1),unit_vector(v11)))
					#calculating_intention(prev_val_obj,val_,x)
					#prev_theta1_obj[x] = theta1_obj[x]
					#prev_theta2_obj[x] = theta2_obj[x]
					'''if unit_v_[x] == -1:
						unit_v[x] = (int) unit_vector_direction(v1)
						unit_v_[x] = 1
					theta1_obj[x] =  (int) unit_vector_direction(v1)'''
					#theta1_obj[x] = (int)(180 * angle_between(v1,v2,True)/np.pi) #calculating the angle between first two vectors in degree
					#theta1_obj[x] = (int)(math.degrees(theta1_obj[x]))
					#theta2_obj[x] = (int)(180 * angle_between(v3,v4,True)/np.pi) #calculating the angle between last two vectors in degree	
					#theta2_obj[x] = (int)(math.degrees(theta2_obj[x]))
					#val_obj[x] = intention_object(theta1_obj[x],unit_v[x])
					#theta2_obj[x] = unit_v[x] - theta1_obj[x]
					#print "%s %d val"%(object_list[x],val_obj[x])
					with open("exampleval.txt", "a") as f:
						f.write("\nvalue %d %s val_ %f %d %d %f %f %f %f %f %f %f %f %f"%(count,object_list[x],val_obj[x],hand.center_x_2d,hand.center_y_2d,hand_x_avg,hand_y_avg,hand_z_avg,hand_.center_x,hand_.center_y,hand_.center_z,obj1_[x].center_x,obj1_[x].center_y,obj1_[x].center_z))
			



			
			
			
			#print "%d\n"%(count)
			m= [ i for i,v in enumerate(val_obj) if v==max(val_obj) ]
			m_= [ i for i,v in enumerate(prev_val_obj) if v==max(prev_val_obj) ]
			#obj_chance2 = [c1,c2,c3,c4,c5,c6,c7]

			for i in range(0,len(m_)):
				for j in range(0,len(m)):
					if m_[i] == m[j] and val_obj[m_[i]]!=-2 and val_obj[m_[i]]!=-1:
						c(m_[i])
						#obj_chance2[m[j]] = obj_chance2[m[j]] + 1

	            
			#print(obj_chance2)

			for i in range(0,len(m)):
				if val_obj[m[i]]!=-2 and val_obj[m[i]]!=-1 and val_obj[m[i]]!=0:
					c(m[i])
					#obj_chance2[m[i]] = obj_chance2[m[i]] + 1
    
    
			#print(obj_chance2)
			obj_chance2 = [c1,c2,c3,c4,c5,c6,c7]
			with open("exampleval_chance.txt", "a") as f:
				f.write("\n noti_calleeeeeeee %d chance %s"%(count,obj_chance2))

			

			
					
		
		
			hand_x_sum = 0;
			hand_y_sum = 0;
			hand_z_sum = 0;

		
		count = count + 1
		
	notify_callback()
		

def notify_callback():
	global started,object_started_flag,done,obj_chance2,obj_c 
	#chance_list = [cup_chance,scissor_chance,clock_chance,bear_chance,ball_chance]
	#c2 = 14 #jsut to make the burger shut down
	obj_chance = [c1,c2,c3,c4,c5,c6,c7]
	
	#obj_chance = obj_chance2
	with open("exampleval_chance.txt", "a") as f:
		f.write("\n noti_call %d chance %s"%(count,obj_chance))
	max_first = max(obj_chance)
	max_first_index = obj_chance.index(max(obj_chance))
	obj_chance.pop(max_first_index)
	obj_chance.insert(max_first_index,0)
	max_second = max(obj_chance)
	max_second_index = obj_chance.index(max_second)
	diff = max_first - max_second
	if object_started_flag == 0 and max_first - max_second >15:
		object_started_flag = 1
		started[max_first_index]=1
		obj_c[max_first_index] = 100
	# if max_first_index == 1:
	# 	started[max_first_index] = 0
	with open("example5.txt", "a") as f:
		f.write("\nnotify_callback chance_value %d %d %d %d %d %d\nobject started flag: %d"%(c1,c2,c3,c4,c5,count,object_started_flag))
	with open("example5.txt", "a") as f:
		f.write("\nnotify_callback started_value %d %d %d %d %d %d"%(started[0],started[1],started[2],started[3],started[4],count))
	with open("example5.txt", "a") as f:
		f.write("\nnotify_callback done_value %d %d %d %d %d %d"%(done[0],done[1],done[2],done[3],done[4],count))

	for x in range(0, l):
		if started[l]==1:
			print("\nobject name\n\n\n\n\n\n\n\n %s"%(object_list[x]))
			with open("example5.txt", "a") as f:
				f.write("\nobject name %s"%(object_list[x]))


	cup_msg = ObjStatus()
	cup_msg.chance = obj_c[2]
	cup_msg.started = started[2]
	cup_msg.done = done[2]
	tea_pot_msg = ObjStatus()
	tea_pot_msg.chance = obj_c[0]
	tea_pot_msg.started = started[0]
	tea_pot_msg.done = done[0]
	# sugar_msg = ObjStatus()
	# sugar_msg.chance = obj_chance[2]
	# sugar_msg.started = started[2]
	# sugar_msg.done = done[2]
	# burger_msg = ObjStatus()
	# burger_msg.chance = obj_c[1]
	# burger_msg.started = started[1]
	# burger_msg.done = done[1]
	burger_msg = ObjStatus()
	burger_msg.chance = 0
	burger_msg.started = 0
	burger_msg.done = 0
	# sandwich_msg = ObjStatus()
	# sandwich_msg.chance = obj_chance[4]
	# sandwich_msg.started = started[4]
	# sandwich_msg.done = done[4]
	apple_msg = ObjStatus()
	apple_msg.chance = obj_c[1]
	apple_msg.started = started[1]
	apple_msg.done = done[1]
	# apple_msg = ObjStatus()
	# apple_msg.chance = obj_chance[5]
	# apple_msg.started = started[5]
	# apple_msg.done = done[5]
	# orange_msg = ObjStatus()
	# orange_msg.chance = obj_chance[6]
	# orange_msg.started = started[6]
	# orange_msg.done = done[6]


	
	cup_status_pub.publish(cup_msg)
	tea_pot_status_pub.publish(tea_pot_msg)
	#sugar_status_pub.publish(sugar_msg)
	burger_status_pub.publish(burger_msg)
	#sandwich_status_pub.publish(sandwich_msg)
	apple_status_pub.publish(apple_msg)
	#orange_status_pub.publish(orange_msg)


	    # server = InteractiveMarkerServer("simple_marker")
	    # int_marker = InteractiveMarker()
	    # int_marker.header.frame_id = "base_link"
	    # int_marker.name = "my_marker"
	    # box_marker = Marker()
	    # box_marker.type = Marker.SPHERE
	    # box_marker.scale.x = 0.45
	    # box_marker.scale.y = 0.45
	    # box_marker.scale.z = 0.45
	    # box_marker.color.r = 0.0
	    # box_marker.color.g = 0.5
	    # box_marker.color.b = 0.5
	    # box_marker.color.a = 1.0


	    # # create a non-interactive control which contains the box
	    # box_control = InteractiveMarkerControl()
	    # box_control.always_visible = True
	    # box_control.markers.append( box_marker )

	    # # add the control to the interactive marker
	    # int_marker.controls.append( box_control )

	    # # create a control which will move the box
	    # # this control does not contain any markers,
	    # # which will cause RViz to insert two arrows
	    # # rotate_control = InteractiveMarkerControl()
	    # # rotate_control.name = "move_x"
	    # # rotate_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS

	    # # # add the control to the interactive marker
	    # # int_marker.controls.append(rotate_control);

	    # # add the interactive marker to our collection &
	    # # tell the server to call processFeedback() when feedback arrives for it
	    # server.insert(int_marker, processFeedback)

	    # # 'commit' changes and send to all clients
	    # server.applyChanges()



def image_callback(msg_img):
	global draw_box,cup_chance,scissor_chance,clock_chance,bear_chance,hand_fix_flag,distance,ball_chance,obj_chance,obj1_,obj2_ 
	global started,done,object_list
	bridge = CvBridge()
	cv_image = bridge.imgmsg_to_cv2(msg_img, 'bgr8')

	#cv2.circle(cv_image,(330,70),5,[0,0,255],-1)

	cv2.circle(cv_image,(330,180),5,[0,0,255],-1) #burger

	cv2.circle(cv_image,(440,285),5,[0,0,255],-1) #cup

	cv2.circle(cv_image,(270,299),5,[0,0,255],-1) #teapot


	cv2.circle(cv_image,(500,200),5,[0,0,255],-1) #apple
	font = cv2.FONT_HERSHEY_SIMPLEX
	'''with open("example4.txt", "a") as f:
		f.write("\nimage_callback %d %d %d %d %d flag=%d"%(count,cup_chance,scissor_chance,clock_chance,bear_chance,hand_fix_flag))'''
	if done[0] == 0 and started[0] == 1:
		cv2.putText(cv_image,'Tea Pot',(270,299), font, 1, (200,255,155), 2)
		cv2.circle(cv_image,(270,299),15,[0,0,255],-1)
	# elif done[1] == 0 and started[1] == 1:
	# 	cv2.putText(cv_image,'Burger',(330,180), font, 1, (200,255,155), 2)
	# 	cv2.circle(cv_image,(330,180),15,[0,0,255],-1)
	elif done[1] == 0 and started[1] == 1:
		cv2.putText(cv_image,'Apple',(500,200), font, 1, (200,255,155), 2)
		cv2.circle(cv_image,(500,200),15,[0,0,255],-1)
	elif done[2] == 0 and started[2] == 1:
		cv2.putText(cv_image,'Cup',(440,285), font, 1, (200,255,155), 2)
		cv2.circle(cv_image,(440,285),15,[0,0,255],-1)
	#cv2.circle(cv_image, (0.405885,-0.545961,1.825000 ), 30, (255,255,255), 2)
	'''for x in range(0, l):
		if done[x] == 0 and started[x] ==1:
			cv2.rectangle(cv_image, (obj1_[x].center_x - .0020,obj1_[x].center_y - .002), (obj1_[x].center_x + .002,obj1_[x].center_y + .002), (255,255,255), 4, 8, 0)
	
	if done[0] ==0 and started[0]==1:
			#cv2.rectangle(cv_image, (cup_loc_save.xmin,cup_loc_save.ymin), (cup_loc_save.xmax,cup_loc_save.ymaxx), (255,255,255), 4, 8, 0)
			cv2.rectangle(cv_image, (obj1_[0].center_x - 10,obj1_[0].center_y - 10), (obj1_[0].center_x + 10,obj1_[0].center_y + 10), (255,255,255), 4, 8, 0)
	
	elif done[1] ==0 and started[1]==1:
			
			cv2.rectangle(cv_image, (scissor_loc_save.xmin,scissor_loc_save.ymin), (scissor_loc_save.xmax,scissor_loc_save.ymaxx), (255,255,255), 4, 8, 0)
	
	elif done[2] ==0 and started[2]==1:
			
	
			cv2.rectangle(cv_image, (clock_loc_save.xmin,clock_loc_save.ymin), (clock_loc_save.xmax,clock_loc_save.ymaxx), (255,255,255), 4, 8, 0)
	elif done[3] ==0 and started[3]==1:
			
			cv2.rectangle(cv_image, (bear_loc_save.xmin,bear_loc_save.ymin), (bear_loc_save.xmax,bear_loc_save.ymaxx), (255,255,255), 4, 8, 0)
	elif done[4] ==0 and started[4]==1:
			
			cv2.rectangle(cv_image, (ball_loc_save.xmin,ball_loc_save.ymin), (ball_loc_save.xmax,ball_loc_save.ymaxx), (255,255,255), 4, 8, 0)'''
	
	'''elif draw_box == 0:
		cv2.rectangle(cv_image, (100,100), (200,200), (255,255,255), 1, 8, 0)'''
	cv2.imshow("Image window", cv_image)
	cv2.waitKey(1)
def get_intention():
	

    	
	sub_center = rospy.Subscriber('hand_bounding_box_center', bounding_box_calculated_center, center_callback) #subscribing the topic which publishing hand center in 3d   
    	sub_img = rospy.Subscriber('/camera/rgb/image_color', Image, image_callback)

	
	
	
	




   	 


def usage():
    return "%s [obj_name]"%sys.argv[0]

if __name__ == "__main__":
    '''if len(sys.argv) == 1:
       # obj_name = sys.argv[1]
	obj_name = "person"
    else:
        print usage()
        sys.exit(1)'''
  
    rospy.init_node('get_obj_loc')
    get_intention()

    	 
	
 
    rospy.spin()	
    	
	
	
