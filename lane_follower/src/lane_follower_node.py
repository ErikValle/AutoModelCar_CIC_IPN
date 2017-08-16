#! /usr/bin/python
# it requires "chmod +x mypythonscript.py" to be called by ROS
##############################################################
# Lane follow vS2
# Autor: Cesar Bravo
# 29/03/2017
##############################################################

import rospy
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Int16
import cv2
import numpy as np
import time
import math
from pendCalc import pend
from numpy import *

#Constants

MAX_DIST = 50
MAX_SPEED_BACK = 300
MAX_SPEED_FOR = -1000
MAX_STEER_LEFT = 20
MAX_STEER_RIGHT = 160
NUM_CUADROS = 3
LANE_WIDTH = 4
TOL = 3
bridge = CvBridge()


def myShut():

	global pubVel
	global pubDir
	
	print('Lane follow node stopped')
	pubVel.publish(0)
	pubDir.publish(98)

def dirControl(err, tht, errAnt):
	
	global MAX_VEL 

	if tht>110 or tht<90:
		pe = 1.25
		pp = 1
		MAX_VEL = -600
	else:	
		pe = 0.3
		pp = 1
		MAX_VEL = -700

	sal = err*pe + tht*pp + (err-errAnt)*1
	
	return int(sal)


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
	global angDir
	global vel	
	global err	
	global tht
        global maxVel
        global errAnt
	global MAX_VEL
	global frCnt 
	global antCent
	

	e1 = cv2.getTickCount()

        #1. Convert data received to numpy array
        image = np.asarray(cv2_img) # 480x640x3	
	hght, wdth = image.shape[:2] #360x320	
	
	thd = cv2.inRange(image, 140, 190)
	#thd = thd/255

	# 3. Color transform ---------------------------------------------------
	heatFlag = True
	'''if frCnt<NUM_CUADROS:
		heat += thd
		frCnt += 1
	else:
		im_heat = cv2.inRange(heat,2,255)
		heat = np.zeros((hght,wdth), dtype=np.uint8)
		frCnt = 0
		heatFlag = True'''

	#2. Lane center detection ---------------------------------------------
	if heatFlag == True:

		i = hght-10
		jI = antCent
		jD = antCent
		point = antCent

		cents = []
		iniIzq = []
		iniDer = []

		cents.append((antCent,i))
		iniIzq.append((antCent-MAX_DIST,i))
		iniDer.append((antCent+MAX_DIST,i))

		cv2.circle(image,cents[-1],3,155,-1)
		cv2.circle(image,iniIzq[-1],3,255,-1)
		cv2.circle(image,iniDer[-1],3,55,-1)

		while i > hght*(0.8):	

			cont = 0
			pc = []

			#Lado derecho
			centDerFlag = False
			while cents[-1][0] <= jD < wdth: 
				if thd[i,jD] != 0:
					cont += 1
					pc.append(jD)
				else:	
					if LANE_WIDTH-TOL < cont < LANE_WIDTH+TOL :
						x = abs(iniDer[-1][0]-pc[0])
						y = abs(iniDer[-1][1]-i)
						if x<(20*TOL) and y<(10*TOL):
							iniDer.append((pc[0],i))
							cv2.circle(image,iniDer[-1],1,50,-1)
							point = iniDer[-1][0]-TOL
							centDerFlag = True
							jD = wdth
						else:
							point = cents[-1][0]
					else:
						point = cents[-1][0]

					cont = 0
					pc = []
				jD += 1

			jD = point

			#Lado Izquierdo
			centIzqFlag = False
			while wdth> cents[-1][0] >= jI > 0: 
				if thd[i,jI] != 0:
					cont += 1
					pc.append(jI)
				else:	
					if LANE_WIDTH-TOL < cont < LANE_WIDTH+TOL :
						x = abs(iniIzq[-1][0]-pc[0])
						y = abs(iniIzq[-1][1]-i)
						if x<(20*TOL) and  y<(25*TOL):
							iniIzq.append((pc[0],i))
							cv2.circle(image,iniIzq[-1],1,250,-1)
							point = iniIzq[-1][0]+ 3*TOL
							centIzqFlag = True
							jI = 0
						else:
							point = cents[-1][0]
					else:
						point = cents[-1][0]

					cont = 0
					pc = []
				jI -= 1

			jI = point

			if centDerFlag == True:

				cent = iniDer[-1][0]-MAX_DIST
				cents.append((cent,i))
				antCent =  cents[1][0]
				

			elif centIzqFlag == True:
				
				cent = iniIzq[-1][0]+MAX_DIST
			 	cents.append((cent,i))
				antCent =  cents[1][0]

			
			cv2.circle(image,cents[-1],1,155,-1)			
			i -= 4		
			
	
	# STEERING calculation and publication ---------------------------------
		p = None
		if len(iniDer)>8:
			p = pend(iniDer[1:])
			desv = -MAX_DIST
		
		elif len(iniIzq)>5:
			p = pend(iniIzq[1:])
			desv = MAX_DIST 
		
		if p != None:

			err = wdth/2-cents[1][0]		

			x1 = linspace(hght*(0.8),hght-10,20)
			y1 = poly1d(p)
			for i in x1:

				cv2.circle(image,(int(y1(i))+desv,int(i)),1,255,-1)
			
			thtCalc = 98 + int(57*arctan(p[0]))
			tht = thtCalc			


		calcDir = dirControl(err, tht, errAnt)
		if calcDir>angDir+3:
			angDir += 3
		elif calcDir<angDir-3:
			angDir -= 3

		errAnt = err
		print('calcDir: ' + str(calcDir))


	# SPEED alculation and publication -------------------------------------
		maxVel = int(MAX_VEL*np.e**(-1*(0.002)*abs(err*10)))
		print('MaxVel: ' + str(maxVel))

		if vel>maxVel+9:
			vel -= 2

		elif vel<=maxVel:
			vel += 5

	# Sevomotor saturation -------------------------------------------------
		if angDir>MAX_STEER_RIGHT:
			angDir = MAX_STEER_RIGHT

		elif angDir<MAX_STEER_LEFT:
			angDir = MAX_STEER_LEFT
	
		pubDir.publish(angDir)
		pubVel.publish(vel)

	
	# Compress image to pub ------------------------------------------------
        	cropImage = CompressedImage()
        	cropImage.header.stamp = rospy.Time.now()
        	cropImage.format = "jpeg"
        	cropImage.data = np.array(cv2.imencode('.jpg',image)[1]).tostring()
        	pub.publish(cropImage)

	# Print stats ----------------------------------------------------------
	e2 = cv2.getTickCount()	
    	t = (e2 - e1)/cv2.getTickFrequency()	
	
	print('error: '+str(err)+' | Theta:'+str(tht))
	print('angDir:'+str(angDir)+'| vel: '+str(vel)+'| frame time:'+str(t))
	print('------------------------------------------------------block end')
        

def main():

    global pub
    global pubDir
    global pubVel
    global angDir
    global vel
    global err
    global tht
    global errAnt
    global MAX_VEL
    global frCnt
    global antCent
   
    antCent = 128
    size = 432,256
    frCnt = 0
    angDir = 98
    vel = 0
    err = 0
    tht = 98.0
    errAnt = 0.0
    MAX_VEL = -600

    rospy.init_node('lane_follower')
    image_topic = "/img_prepros/compressed"
    rospy.Subscriber(image_topic, CompressedImage, image_callback,queue_size=1)
    pub = rospy.Publisher('/lane_follower/compressed',CompressedImage, queue_size=1)
    pubDir = rospy.Publisher('/lane_follower/steering', Int16, queue_size=1)
    pubVel = rospy.Publisher('/lane_follower/speed', Int16, queue_size=1)
    #pubDir = rospy.Publisher('/manual_control/steering', Int16, queue_size=1)
    #pubVel = rospy.Publisher('/manual_control/speed', Int16, queue_size=1)
    rospy.on_shutdown(myShut)
    rospy.spin()

if __name__ == '__main__':
    main()

