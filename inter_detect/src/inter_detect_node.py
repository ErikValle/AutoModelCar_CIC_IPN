#! /usr/bin/python
# it requires "chmod +x mypythonscript.py" to be called by ROS
##############################################################
# Intersection line detector v1
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

#Constant
LINE_WIDTH = 8
TOL = 4

bridge = CvBridge()


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

	global dist
	global ang
	# Processing function begins
	e1 = cv2.getTickCount()

        # 1. Convert data received to numpy array
        image = np.asarray(cv2_img) # 480x640x3
	scld = cv2.resize(image,None,fx=0.6,fy=0.6,interpolation=cv2.INTER_AREA)

	# 2. Filtering and Binarization ----------------------------------------
	thd = cv2.inRange(scld, 140, 190)
	hght, wdth = thd.shape[:2] #216x192
	#print(hght,wdth)

	# 3. Intersection line detection ---------------------------------------
	i = hght-2
	j = int(wdth*(0.35))

	point = hght-1
	ini = []

	while j < wdth*(0.7):	

		cont = 0
		pc = []
		#Lado Izquierdo
		while hght*(0.5) < i < hght:
			if thd[i,j] != 0:
				cont += 1
				pc.append(i)

			else:
				if LINE_WIDTH-TOL < cont < LINE_WIDTH+TOL:
					if abs(point - pc[0])<7:
						ini.append((pc[0],j))
						cv2.circle(scld,(j,pc[0]),1,250,-1)
						point = pc[0]+TOL
						i = 0
					else:
						point = pc[0] + TOL
						i = 0
				cont = 0
				pc = []		

			i -= 1

		i = hght - 5
		j += 2		
			
	# Distance and angle calculation and publication -----------------------
	p = None
	if len(ini)>10:
		p = pend(ini)

	else: 
		dist = 200

	if p != None:

		x1 = linspace(wdth*0.25,wdth*0.7,10)
		y1 = poly1d(p)
		for i in x1:

			cv2.circle(scld,(int(i),int(y1(i))),1,0,-1)
		
		thtCalc = int(57*arctan(p[0]))
		ang = thtCalc
		dist = int(hght - int(mean(ini, axis=0)[0]))

	if ang>-15 and ang<15:

		pubDist.publish(dist)
		pubAng.publish(ang)
	

	
	# Compress image to pub ------------------------------------------------
        cropImage = CompressedImage()
        cropImage.header.stamp = rospy.Time.now()
        cropImage.format = "jpeg"
        cropImage.data = np.array(cv2.imencode('.jpg',scld)[1]).tostring()
        pub.publish(cropImage)

	# Print stats ----------------------------------------------------------
	e2 = cv2.getTickCount()	
    	t = (e2 - e1)/cv2.getTickFrequency()	
	
	print('angle: '+str(ang)+'| distance: '+str(dist)+'| frame time:'+str(t))
	print('----------------------------------------------------block end')
        

def main():

    global pub
    global pubDist
    global pubAng
    global ang
    global dist
   

    ang = 180
    dist = 100

    rospy.init_node('inter_line')
    image_topic = "/img_prepros/compressed"
    rospy.Subscriber(image_topic,CompressedImage,image_callback,queue_size=1)
    pub = rospy.Publisher('/inter_line/compressed',CompressedImage, queue_size=1)
    pubDist = rospy.Publisher('/inter_line/dist', Int16, queue_size=1)
    pubAng = rospy.Publisher('/inter_line/ang', Int16, queue_size=1)
    rospy.spin()

if __name__ == '__main__':
    main()

