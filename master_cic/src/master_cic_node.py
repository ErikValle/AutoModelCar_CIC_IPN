#! /usr/bin/python
# it requires "chmod +x mypythonscript.py" to be called by ROS
##############################################################
# Master v2
# Autor: Cesar Bravo
# 29/03/2017
##############################################################

import rospy
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Int16
from std_msgs.msg import Float32
from std_msgs.msg import String
import cv2
import numpy as np
import time
import math
from pendCalc import pend
from numpy import *

#Constants
KEEP_DIST = 0.8
bridge = CvBridge()

# Power down function
def myShut():
	
	global pubDir
	global pubAng

	pubVel.publish(0)
	pubDir.publish(98)
	time.sleep(0.5)
	print('Aufwiedersehen!')
	
# Steering callback function
def steering_callback(ros_data):

	global laneFollowSteering

	laneFollowSteering = ros_data.data

# Speed callback function
def speed_callback(ros_data):

	global laneFollowSpeed

	laneFollowSpeed = ros_data.data

# The angle of intersection line if detected detected
def line_ang_callback(ros_data):
	
	global line_ang
	
	line_ang = ros_data.data

# The distance to the intersection line  if detected detected
def line_dist_callback(ros_data):

 	global line_dist
	
	line_dist = ros_data.data

# The angle of the obstacle if detected
def scan_ang_callback(ros_data):
	
	global scan_ang
	
	scan_ang = ros_data.data

# The distance to the obstacle if detected
def scan_dist_callback(ros_data):

 	global scan_dist
	
	scan_dist = ros_data.data

# The velocity regulator function
def valCatch(vel, cval):

	if vel>cval+9:
		vel -= 2
	elif vel<cval:
		vel += 6

	elif cval == 0:
		vel = 0

	return vel

# The Following distance function
def contolDist(scan_dist):	
	
	# Approaching the obstacle (to far)
	if scan_dist > KEEP_DIST + 0.1:
		velCalc = int(scan_dist*(-300)) 
	# Range of distance to keep
	elif scan_dist < KEEP_DIST-0.1 and scan_dist>0.2:
		velCalc = int((1/scan_dist)*(50))
	# Getting away of the obstacle (to close)
	elif KEEP_DIST -0.1 < scan_dist < KEEP_DIST+0.1:
		velCalc = 0
	else: 
		velCalc = 0

	return velCalc

def image_callback(ros_data):
	
	
    try:
        # Convert your ROS Image message to OpenCV2

	# Uncomment the following line to work with raw images
        #cv2_img = bridge.imgmsg_to_cv2(ros_data, "bgr8")

	# Uncoment the following two lines to work with compressed images
	np_arr = np.fromstring(ros_data.data, np.uint8)
	cv2_img = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_GRAYSCALE)

    except CvBridgeError, e:
        print(e)
    else:
	
	# Processing function begins
	global pub
        global pubDir
        global pubVel	
	global angDir
	global vel	
	global laneFollowSteering
	global laneFollowSpeed
	global scan_ang
	global scan_dist
	global stopFlag
	global cnt
	global passFlag
	global lightsAnt
	global pubLight

	e1 = cv2.getTickCount()

        # 1. Convert data received to numpy array
        image = np.asarray(cv2_img) # 480x640x3	

	e1 = cv2.getTickCount()
	image = np.asarray(cv2_img) 

	lights = 'diL'
	# There exist basically three big routines within the program:
	# 1. The OVERCOMING routine---------------------------------------------------------
	if passFlag == True:
		
		# Moving to the left lane 
		if scan_ang>330 or scan_ang<25:

			angDir = 160
			pubLight.publish('diL')
			print('passing left')
			lights = 'le'

		# Retunrning to original lane
		elif 220>scan_ang>85: 

			# Ff the distance to the obstacle is greater than 
			# 0.55 m  and the angle is greater than 85 it 
              		# finishes the OVERCOMING routine
			if scan_dist>0.63:
				lights = 'diL'
				passFlag = False
				print('DONE passing!!')

			# if not, it keeps returning to the left
			else:
				lights = 'ri'
				angDir = 37
				print('passing rigth')

		# Keeps on the rigth lane until it's possible to return
		elif 85>scan_ang>20:
			lights = 'diL'
			angDir = laneFollowSteering

		else: 
			print('B')
			
		vel = -250	
		
	# 2. The LANE_FOLLOWIG routine ---------------------------------------------------------------
	elif stopFlag == False:

		# if there is no intersection line:
		if line_dist>99:
				
			lights = 'diL'
			angDir = laneFollowSteering
			gvel = laneFollowSpeed
			vel = valCatch(vel, gvel)
			print('a: Lane Following')
			
			'''if scan_ang>344 or scan_ang<16:

				angDir = laneFollowSteering
				vel = contolDist(scan_dist)
				print('a: Obstacle Following')
				
				if vel == 0:
					lights = 'stop'
					cnt += 1
					#pubVel.publish(vel)
					print(cnt)	
    					
				else:
					#pubVel.publish(vel)
					lights = 'diL'
					cnt = 0

				if cnt>60:

					cnt = 0
					passFlag = True
					
		
			else:
				lights = 'diL'
				angDir = laneFollowSteering
				gvel = laneFollowSpeed
				vel = valCatch(vel, gvel)
				#pubVel.publish(vel)
				print('a: Lane Following')'''

		elif 60>line_dist>10:	
			angDir = laneFollowSteering
			vel = -200
			#pubVel.publish(vel)
			lights = 'stop'
			print('a: Stopping...')
			time.sleep(0.1)

		elif 10>line_dist:
			vel = 0
			angDir = 98 - 2*line_ang
			#pubVel.publish(vel)
			stopFlag = True	
			lights ='diL'
			print('a: Stopped')
			time.sleep(0.3)
			
		else:
			print('Not Defined 1 : WTF??')

	elif stopFlag == True:
		
		## Existe un bug en la interseccion, cuando esta cruzando, su hay un objeto muy cerca antes de 
		## cruzar la segunda linea, no lo reconoce y se pierde
		print(cnt)
		if (scan_dist<0.2 or 210>scan_ang>30 or 298<scan_ang<330) and cnt>5:
			
			if line_dist<10:
				stopFlag = False
				vel = -300
				cnt = 0
				lights = 'dil'
				print('b: Crossing DONE!')
				time.sleep(1.0)
			else:
				angDir = 98 - 2*line_ang
				vel = -250
				lights = 'ta'
				print('b: Crossing...')
				
		else: 
			if 30>line_dist:
				vel = -100
				pubVel.publish(vel)
				lights ='stop'
				print('b: Getting closer...')
			else:
				vel = 0
				angDir = 98 
				cnt += 1
				lights ='diL'
				print('b: Watting...')

			time.sleep(0.3)
		
	
	pubDir.publish(angDir)	
	pubVel.publish(vel)

	if lights != lightsAnt:
		pubLight.publish(lights)

	lightsAnt = lights
	
	# Compress image to pub ------------------------------------------------
        cropImage = CompressedImage()
        cropImage.header.stamp = rospy.Time.now()
        cropImage.format = "jpeg"
        cropImage.data = np.array(cv2.imencode('.jpg',image)[1]).tostring()
        pub.publish(cropImage)

	# Print stats ----------------------------------------------------------
	e2 = cv2.getTickCount()	
    	t = (e2 - e1)/cv2.getTickFrequency()
	
	print('Stop: ' +str(stopFlag))
	print('scan_dist: '+str(scan_dist)+' | scan_ang: '+str(scan_ang))
	print('line_dist: '+str(line_dist)+' | angDir:'+str(angDir)+' | vel: '+str(vel)+' | frame time:'+str(t))
	print('------------------------------------------------------block end')
        

def main():

    global pub
    global pubDir
    global pubVel
    global pubLight

    global angDir
    global vel
    global cnt
    global passFlag
   
    global laneFollowSteering
    global laneFollowSpeed
    global line_ang
    global line_dist
    global stopFlag
    global scan_dist
    global scan_ang
    global lightsAnt

    angDir = 98
    vel = 0
    laneFollowSteering = 100
    laneFollowSpeed = 0
    scan_dist = 0.1
    scan_ang = 220
    line_ang = 180
    line_dist = 100
    stopFlag = False
    passFlag = False
    cnt = 0
    lightsAnt = 'diL'

    rospy.init_node('master_cic')

    image_topic = "/img_prepros/compressed"
    rospy.Subscriber(image_topic, CompressedImage, image_callback,queue_size=1)

    steering_topic = "/line_follower/steering"
    speed_topic = "/line_follower/speed"
    rospy.Subscriber(steering_topic, Int16, steering_callback, queue_size = 1)
    rospy.Subscriber(speed_topic, Int16, speed_callback, queue_size = 1)

    line_ang_topic = "/inter_line/ang"
    line_dist_topic = "/inter_line/dist"  
    rospy.Subscriber(line_ang_topic, Int16, line_ang_callback, queue_size = 1)
    rospy.Subscriber(line_dist_topic, Int16, line_dist_callback, queue_size = 1)

    line_ang_topic = "/scan_follow/angle"
    line_dist_topic = "/scan_follow/dist"  
    rospy.Subscriber(line_ang_topic, Int16, scan_ang_callback, queue_size = 1)
    rospy.Subscriber(line_dist_topic, Float32, scan_dist_callback,queue_size=1)

    pub = rospy.Publisher('/master/compressed',CompressedImage, queue_size=1)
    pubDir = rospy.Publisher('/manual_control/steering', Int16, queue_size=1)
    pubVel = rospy.Publisher('/manual_control/speed', Int16, queue_size=1)
    pubLight = rospy.Publisher('/manual_control/lights', String, queue_size=1)

    rospy.spin()
    rospy.on_shutdown(myShut)
    

if __name__ == '__main__':
    main()

