# import the necessary packages
from shapedetector import ShapeDetector
from colorlabeler import ColorLabeler
from std_msgs.msg import Int64MultiArray
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import MultiArrayDimension
from geometry_msgs.msg import PointStamped
import numpy as np
import argparse
import imutils
import cv2
import numpy as np
import math
import tf
import rospy

def Blue_Filtering(raw_im,blue_center,image_depth):
	BlueImage = raw_im.copy()
	lower_blue = np.array([64, 0, 0])
	upper_blue = np.array([227, 61, 61])
	mask = cv2.inRange(BlueImage, lower_blue, upper_blue)
	result = cv2.bitwise_and(BlueImage, BlueImage, mask = mask)
	resized = imutils.resize(result, width=300)
	ratio = result.shape[0] / float(resized.shape[0])
	gray = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)
	blurred = cv2.GaussianBlur(gray, (5, 5), 0)
	thresh = cv2.threshold(blurred, 0, 170, cv2.THRESH_BINARY)[1]
	# find contours in the thresholded image and initialize the
	# shape detector
	cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL,	cv2.CHAIN_APPROX_SIMPLE)
	cnts = imutils.grab_contours(cnts)
	sd = ShapeDetector()
	colour = "person "
	i=0
	blue_center.layout.dim.append(MultiArrayDimension())
	blue_center.layout.dim[0].label="blue_centers: cX,cY"
	blue_center.layout.dim[0].size = 2
	blue_center.layout.dim[0].stride =0
	blue_center.layout.data_offset = 0
	blue_center.data = []
	for c in cnts:
		# compute the center of the contour, then detect the name of the
		# shape using only the contour
		M = cv2.moments(c)
		cX = int((M["m10"] / M["m00"]) * ratio)
		cY = int((M["m01"] / M["m00"]) * ratio)
		i+=1
		shape = sd.detect(c,colour,i)
		# multiply the contour (x, y)-coordinates by the resize ratio,
		# then draw the contours and the name of the shape on the image
		c = c.astype("float")
		c *= ratio
		c = c.astype("int")
		distance = image_depth[cY][cX]
		if (cX not in blue_center.data) or (cY not in blue_center.data):
			blue_center.data=blue_center.data+[cX,cY,distance]
		else:
			pass
		#if colour_dict['blue'] == 'None':
		#	colour_dict['blue'] = [[cX,cY]]
		#else:
			#colour_dict['blue']=[colour_dict['blue'],[cX, cY]]
		#	colour_dict['blue'].append([cX, cY])
		cv2.drawContours(raw_im, [c], -1, (0, 255, 0), 2)
		cv2.circle(raw_im, (cX, cY), 7, (255, 255, 255), -1)
		#print(distance)
		cv2.putText(raw_im, shape, (cX, cY), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
	#print(blue_center)
	return raw_im,blue_center


def Red_Filtering(raw_im,red_center,image_depth):
  	#cv2.imshow("image",im)
  	#cv2.imshow("Imag", Image)
	RedImage = raw_im.copy()
	lower_red = np.array([0, 0, 56])
	upper_red = np.array([10, 96, 195])
	mask = cv2.inRange(RedImage, lower_red, upper_red)
	result = cv2.bitwise_and(RedImage, RedImage, mask = mask)
	resized = imutils.resize(result, width=300)
	ratio = result.shape[0] / float(resized.shape[0])
	gray = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)
	blurred = cv2.GaussianBlur(gray, (5, 5), 0)
	thresh = cv2.threshold(blurred, 15, 55, cv2.THRESH_BINARY)[1]
	#cv2.imshow("thresh", thresh)
  	cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL,	cv2.CHAIN_APPROX_SIMPLE)
  	cnts = imutils.grab_contours(cnts)
  	sd = ShapeDetector()
  	colour = "danger_zones "
	i = 0;
	red_center.layout.dim.append(MultiArrayDimension())
	red_center.layout.dim[0].label="red_centers: cX,cY"
	red_center.layout.dim[0].size = 2
	red_center.layout.dim[0].stride =0
	red_center.layout.data_offset = 0
	red_center.data = []
	#print(image_depth)
  	for c in cnts:
  		# compute the center of the contour, then detect the name of the
  		# shape using only the contour
		i+=1
  		M = cv2.moments(c)
  		cX = int((M["m10"] / M["m00"]) * ratio)
  		cY = int((M["m01"] / M["m00"]) * ratio)
  		shape = sd.detect(c,colour,i)
  		# multiply the contour (x, y)-coordinates by the resize ratio,
  		# then draw the contours and the name of the shape on the image
  		c = c.astype("float")
  		c *= ratio
  		c = c.astype("int")
		#colour_dict["red"].append((cX,cY))
		#distance = image_depth[cX][cY]
		distance = image_depth[cY][cX]
		#print(distance)
		if (cX not in red_center.data) or (cY not in red_center.data):
			red_center.data=red_center.data+[cX,cY,distance]
		else:
			pass
  		cv2.drawContours(raw_im, [c], -1, (0, 255, 0), 2)
  		cv2.circle(raw_im, (cX, cY), 7, (255, 255, 255), -1)
  		cv2.putText(raw_im, shape, (cX, cY), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
	#print(red_center)
	return raw_im,red_center

def Green_Filtering(raw_im,green_center,image_depth):
	GreenImage = raw_im.copy()
	lower_green = np.array([0, 30, 0])
	upper_green = np.array([30, 175, 115])
	mask = cv2.inRange(GreenImage, lower_green, upper_green)
	result = cv2.bitwise_and(GreenImage, GreenImage, mask = mask)
	#cv2.imshow("result",result)
	resized = imutils.resize(result, width=300)
	ratio = result.shape[0] / float(resized.shape[0])
	gray = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)
	#cv2.imshow("gray",gray)
	blurred = cv2.GaussianBlur(gray, (5, 5), 0)
	#thresh = cv2.threshold(blurred, 100, 200, cv2.THRESH_BINARY)[1]
	thresh = cv2.threshold(blurred, 0, 150, cv2.THRESH_BINARY)[1]
	#cv2.imshow("thresh",thresh)
  	cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL,	cv2.CHAIN_APPROX_SIMPLE)
  	cnts = imutils.grab_contours(cnts)
  	sd = ShapeDetector()
  	colour = "way_out "
	i = 0;
	green_center.layout.dim.append(MultiArrayDimension())
	green_center.layout.dim[0].label="green_centers: cX,cY"
	green_center.layout.dim[0].size = 2
	green_center.layout.dim[0].stride =0
	green_center.layout.data_offset = 0
	green_center.data = []
  	for c in cnts:
		# compute the center of the contour, then detect the name of the
  		# shape using only the contour
		i+=1
  		M = cv2.moments(c)
  		cX = int((M["m10"] / M["m00"]) * ratio)
  		cY = int((M["m01"] / M["m00"]) * ratio)
  		shape = sd.detect(c,colour,i)
  		c = c.astype("float")
  		c *= ratio
  		c = c.astype("int")
		distance = image_depth[cY][cX]
		if (cX not in green_center.data) or (cY not in green_center.data):
			green_center.data=green_center.data+[cX,cY,distance]
		else:
			pass
		#print(green_center)
		#print(colour_dict)
		#if colour_dict['green'] == 'None':
		#	colour_dict['green'] = [[cX,cY]]
		#else:
		#	colour_dict['green'].append([cX, cY])
  		cv2.drawContours(raw_im, [c], -1, (0, 255, 0), 2)
  		cv2.circle(raw_im, (cX, cY), 7, (255, 255, 255), -1)
  		cv2.putText(raw_im, shape, (cX, cY), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
		#cv2.imshow("image",raw_im)
	#print(green_center)
	#cv2.imshow("raw_im",raw_im)
	#cv2.waitKey(0)
	return raw_im,green_center

def direction(center_data,matrixK,colour):
	#print(matrixK)
	a = len(center_data.data)
	direction_data = Float64MultiArray()
	direction_data.layout.dim.append(MultiArrayDimension())
	direction_data.layout.dim[0].label= colour +": dX,dY"
	direction_data.layout.dim[0].size = a/2
	direction_data.layout.dim[0].stride =0
	direction_data.layout.data_offset = 0
	direction_data.data = []
	i=0
	t= True
	k_matrix = np.array([[matrixK[0],matrixK[1],matrixK[2]],[matrixK[3],matrixK[4],matrixK[5]],[matrixK[6],matrixK[7],matrixK[8]]])
	while i<a:
		center_vector = np.array([center_data.data[i],center_data.data[i+1],1])
		center_vector_trans=np.transpose(center_vector)
		#print(center_vector_trans)
		k_matrix_inv =np.linalg.inv(k_matrix)
		#print(k_matrix_inv)
		#print(colour)
		direction = np.linalg.multi_dot([k_matrix_inv,center_vector_trans])
		#print(direction)
		distancia = center_data.data[i+2]
		if math.isnan(distancia):
			distancia=1
		vector_pose = np.linalg.multi_dot([direction,distancia])
		#print(vector_pose)
		#direction_data.data = direction_data.data + [direction[0]] + [direction[1]] + [direction[2]]
		direction_data.data = direction_data.data + [vector_pose[0]]+ [vector_pose[1]]+[vector_pose[2]]
		i=i+3
	return direction_data


def rplidar_map_transform_(colour,data_0,data_1,data_2,laser,angle_increment):
	print(colour)
	colour_pt = PointStamped()
	lidar_colour_pt = PointStamped()
	lidar_colour_pt_ = PointStamped()
	colour_pt.header.seq = laser.header.seq
	colour_pt.header.stamp = laser.header.stamp
	colour_pt.header.frame_id = "camera_depth_optical_frame"
	listener = tf.TransformListener()
	#listener2 = tf.TransformListener()
	#print(data)
	colour_pt.point.x = round(data_0,3)
	colour_pt.point.y = round(data_1,3)
	colour_pt.point.z = round(data_2,3)
	try:
 		lidar_colour_pt = listener.transformPoint("rplidar_frame",colour_pt)
	except:
		print("1")
 		pass
	angle =math.atan2(lidar_colour_pt.point.y,lidar_colour_pt.point.x)
	angle_increment = angle_increment
	distance = 0
	if (0 <= angle) and (angle<=math.pi/2.0):
 		pos = int(round(angle/angle_increment,0))
 		laser_data = laser
 		distance = laser_data.ranges[pos]
	elif (angle<0) and (abs(angle)<=math.pi/2.0):
 		pos = 399 - int(round(abs(angle)/angle_increment,0))
 		laser_data = laser
 		distance = laser_data.ranges[pos]
	else:
 		print("else")
		print("angle",angle*180/math.pi)
		print(i)
		lidar_colour_pt.header.stamp = rospy.Time(0)
		lidar_colour_pt.header.frame_id = "rplidar_frame"
	try:
		if (colour_pt.point.x == 1.0):
			lidar_colour_pt.point.x *= distance
			lidar_colour_pt.point.y *= distance
 			lidar_colour_pt.point.z *= distance
	 		#print("point_x=1",lidar_colour_pt.point.x )
 		else:
			lidar_colour_pt.point.x = (lidar_colour_pt.point.x/lidar_colour_pt.point.x)*distance
			lidar_colour_pt.point.y = (lidar_colour_pt.point.y/lidar_colour_pt.point.x)*distance
			lidar_colour_pt.point.z = (lidar_colour_pt.point.z/lidar_colour_pt.point.x)*distance
			print(lidar_colour_pt)
	except:
		print("cannot convert to rplidar")
		pass

	return lidar_colour_pt_


def rplidar_map_transform(colour,data,laser,angle_increment):
	"""print(colour)
	colour_pt = PointStamped()
	lidar_colour_pt = PointStamped()
	lidar_colour_pt_ = PointStamped()
	colour_pt.header.seq = laser.header.seq
	colour_pt.header.stamp = laser.header.stamp
	colour_pt.header.frame_id = "camera_depth_optical_frame"
	l =len(data.data)
	listener = tf.TransformListener()
	listener2 = tf.TransformListener()
	i=0
	while i<l:""
		colour_pt.point.x = round(data.data[i],3)
		colour_pt.point.y = round(data.data[i+1],3)
	 	colour_pt.point.z = round(data.data[i+2],3)
	 	try:
		 	lidar_colour_pt = listener.transformPoint("rplidar_frame",colour_pt)
	 	except:
			print("1")
		 	pass
	 	angle =math.atan2(lidar_colour_pt.point.y,lidar_colour_pt.point.x)
	 	angle_increment = angle_increment
	 	distance = 0
	 	if (0 <= angle) and (angle<=math.pi/2.0):
		 	pos = int(round(angle/angle_increment,0))
		 	laser_data = laser
		 	distance = laser_data.ranges[pos]
	 	elif (angle<0) and (abs(angle)<=math.pi/2.0):
		 	pos = 399 - int(round(abs(angle)/angle_increment,0))
		 	laser_data = laser
		 	distance = laser_data.ranges[pos]
	 	else:
		 	print("else")
	 	print("angle",angle*180/math.pi)
	 	print(i)
	 	lidar_colour_pt.header.stamp = rospy.Time(0)
	 	lidar_colour_pt.header.frame_id = "rplidar_frame"
	 	try:
			if (colour_pt.point.x == 1.0):
				lidar_colour_pt.point.x *= distance
				lidar_colour_pt.point.y *= distance
		 		lidar_colour_pt.point.z *= distance
			 	#print("point_x=1",lidar_colour_pt.point.x )
		 	else:
				lidar_colour_pt.point.x = (lidar_colour_pt.point.x/lidar_colour_pt.point.x)*distance
				lidar_colour_pt.point.y = (lidar_colour_pt.point.y/lidar_colour_pt.point.x)*distance
				lidar_colour_pt.point.z = (lidar_colour_pt.point.z/lidar_colour_pt.point.x)*distance
				#print("point_x!=1",lidar_colour_pt.point.x)
		except:
	 		print("2")
			pass
		try:
	 		lidar_colour_pt_=listener.waitForTransform("rplidar_frame", "map", rospy.Time(0), rospy.Duration(10.0))
	 		lidar_colour_pt_=listener.transformPoint("map", lidar_colour_pt)
			return lidar_colour_pt_
		except:
	 		print("cannot pass from rp_lidar_frame to map frame")
	 	pass
		i+=3
	return lidar_colour_pt_"""
