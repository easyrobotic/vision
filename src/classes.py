from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Int64MultiArray
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import MultiArrayDimension
from sensor_msgs.msg import CameraInfo,LaserScan
from geometry_msgs.msg import Vector3,PointStamped
from visualization_msgs.msg import Marker,MarkerArray
from numpy import zeros
import rospy
import cv2
import tf
import math

#from detect_color import Blue_Filtering,Red_Filtering,Green_Filtering,direction
from functions import Blue_Filtering,Red_Filtering,Green_Filtering,direction

class image_converter:
  #Int64MultiArray blue_center()
  def __init__(self):
    self.image_pub = rospy.Publisher("vision/output_image",Image,queue_size=10)
    self.marker_blue = rospy.Publisher("vision/output_data/blue_direction",Float64MultiArray,queue_size=1)
    self.marker_red = rospy.Publisher("vision/output_data/red_direction",Float64MultiArray,queue_size=1)
    self.marker_green = rospy.Publisher("vision/output_data/green_direction",Float64MultiArray,queue_size=1)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/astra/rgb/image_raw",Image,self.callback)
    self.image_sub_depth = rospy.Subscriber("/astra/depth/image_raw",Image,self.callback_depth)
    self.centers = dict()
    self.camera_info = rospy.Subscriber("astra/rgb/camera_info",CameraInfo,self.camera_info_callback)
    self.data_K=[]
    self.image_depth=[]

  def image_centers(self,image,center_dict,blue_center,red_center,green_center):
      image_depth=self.image_depth

      try:
          image,blue_center = Blue_Filtering(image,blue_center,image_depth)
          #print(blue_center)
      except:
          pass
      try:
          image,red_center = Red_Filtering(image,red_center,image_depth)
          #print(red_centers)
      except:
          pass
      try:
          image,green_center = Green_Filtering(image,green_center,image_depth)
      except:
          pass
      return image,center_dict,blue_center,red_center,green_center

  def callback_depth(self,data):
        try:
          cv_image_depth = self.bridge.imgmsg_to_cv2(data, "32FC1")
          self.image_depth=cv_image_depth
        except CvBridgeError as e:
          print(e)

  def callback(self,data):
    try:
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        print(e)
    center_dict = {'blue':'None','red':'None','green':'None'}
    blue_center = Int64MultiArray()
    red_center = Int64MultiArray()
    green_center = Int64MultiArray()
    image,center_dict,blue_center,red_center,green_center = self.image_centers(cv_image,center_dict,blue_center,red_center,green_center)
    try:
        blue_direction = direction(blue_center,self.data_K,"blue")
    except:
        pass
    try:
        red_direction = direction(red_center,self.data_K,"red")
    except:
        pass
    try:
        green_direction = direction(green_center,self.data_K,"green")
    except:
        pass

    try:
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        if blue_direction.layout.dim[0].size != 0:
            self.marker_blue.publish(blue_direction)
        if red_direction.layout.dim[0].size != 0:
            self.marker_red.publish(red_direction)
        if green_direction.layout.dim[0].size != 0:
            self.marker_green.publish(green_direction)
    except CvBridgeError as e:
        print(e)

  def camera_info_callback(self,data):
        data_K = data.K
        self.data_K = data_K


class lidar_converter:
    def __init__(self):
        self.marker_blue = rospy.Subscriber("vision/output_data/blue_direction",Float64MultiArray,self.blue_callback)
        self.marker_red = rospy.Subscriber("vision/output_data/red_direction",Float64MultiArray,self.red_callback)
        self.marker_green = rospy.Subscriber("vision/output_data/green_direction",Float64MultiArray,self.green_callback)
        self.LaserSub = rospy.Subscriber("scan",LaserScan,self.LaserScan_callback)
        self.marker_blue_pub = rospy.Publisher("vision/RPlidar/blue_pose",PointStamped,queue_size=5)
        self.marker_red_pub = rospy.Publisher("vision/RPlidar/red_pose",PointStamped,queue_size=5)
        self.marker_green_pub= rospy.Publisher("vision/RPlidar/green_pose",PointStamped,queue_size=5)
        self.laser = LaserScan()
        self.seq=0
        self.secs=0
        self.nsecs=0
        self.j=0
        #self.marker_red = rospy.Subscriber("vision/output_data/red_direction",Float64MultiArray,self.red_callback)
        self.angle_increment = 0.2
        #self.marker_green = rospy.Subscriber("vision/output_data/green_direction",Float64MultiArray,self.green_callback)

    def blue_callback(self,data):
        print("blue")
        blue_pt = PointStamped()
        lidar_blue_pt = PointStamped()
        lidar_blue_pt_ = PointStamped()
        blue_pt.header.seq = self.laser.header.seq
        blue_pt.header.stamp = self.laser.header.stamp
        blue_pt.header.frame_id = "camera_depth_optical_frame"
        l =len(data.data)
        listener = tf.TransformListener()
        listener2 = tf.TransformListener()
        i=0
        while i<l:
            blue_pt.point.x = round(data.data[i],3)
            blue_pt.point.y = round(data.data[i+1],3)
            blue_pt.point.z = round(data.data[i+2],3)
            print("blue_pt",blue_pt)
            try:
                lidar_blue_pt = listener.transformPoint("rplidar_frame",blue_pt)
                #print(lidar_blue_pt)
            except:
                print("1")
                pass
            angle =math.atan2(lidar_blue_pt.point.y,lidar_blue_pt.point.x)
            #print("angle",angle)
            angle_increment = self.angle_increment
            distance = 0
            #print(angle)
            print(angle)
            print(angle_increment)

            if (0 <= angle) and (angle<=math.pi/2.0):
                #pos_ = angle/angle_increment
                pos = int(round(angle/angle_increment,0))
                #print("pos",pos)
                try:
                    laser_data = self.laser
                    distance = laser_data.ranges[pos]
                except:
                    pass
                #except:
                #pass

                #print("pos1",pos)
                #print("distance",distance)
            elif (angle<0) and (abs(angle)<=math.pi/2.0):
                pos = 399 - int(round(abs(angle)/angle_increment,0))
                laser_data = self.laser
                distance = laser_data.ranges[pos]
            else:
                print("else")
            #print("angle",angle*180/math.pi)
            #print(i)
            lidar_blue_pt.header.stamp = rospy.Time(0)
            lidar_blue_pt.header.frame_id = "rplidar_frame"
            if (blue_pt.point.x == 1.0):
                lidar_blue_pt.point.x *= distance
                lidar_blue_pt.point.y *= distance
                lidar_blue_pt.point.z *= distance
                #print("point_x=1",lidar_blue_pt.point.x )

            else:
                lidar_blue_pt.point.x = (lidar_blue_pt.point.x/lidar_blue_pt.point.x)*distance
                lidar_blue_pt.point.y = (lidar_blue_pt.point.y/lidar_blue_pt.point.x)*distance
                lidar_blue_pt.point.z = (lidar_blue_pt.point.z/lidar_blue_pt.point.x)*distance
                #print("point_x!=1",lidar_blue_pt.point.x)
            print(lidar_blue_pt)
            try:
                lidar_blue_pt_=listener.waitForTransform("rplidar_frame", "map", rospy.Time(0), rospy.Duration(5.0))
                lidar_blue_pt_=listener.transformPoint("map", lidar_blue_pt)
                print(lidar_blue_pt_)
                #print("o")
                #print("3")
                """marker_blue = Marker()
                #print("l")
                marker_blue.header = lidar_blue_pt_.header
                marker_blue.ns = "person"
                marker_blue.id = self.j
                marker_blue.type = Marker.CUBE
                #marker_blue.action = 2
                marker_blue.pose.position.x = round(lidar_blue_pt_.point.x,3)
                marker_blue.pose.position.y = round(lidar_blue_pt_.point.y,3)
                marker_blue.pose.position.z = round(lidar_blue_pt_.point.z,3)
                marker_blue.pose.orientation.x = 0.0
                marker_blue.pose.orientation.y = 0.0
                marker_blue.pose.orientation.z = 0.0
                marker_blue.pose.orientation.w = 1.0
                marker_blue.scale.x = 0.1
                marker_blue.scale.y = 0.2
                marker_blue.scale.z = 0.2
                marker_blue.color.r = 0
                marker_blue.color.g = 0
                marker_blue.color.b = 255
                marker_blue.color.a = 1"""
                #print(marker_blue)
                try:
                    self.marker_blue_pub.publish(lidar_blue_pt_)
                    #rospy.sleep(1)
                    #marker_blue.lifetime = rospy.Duration();
                    #print(marker_blue)
                except:
                    print("blue 3")
                    pass
            except:
                print("blue 4")
                pass
            #print(lidar_blue_pt_)
            i+=3
            self.j+=1


    def LaserScan_callback(self,data):
        print("laser")
        l=len(data.ranges)
        ranges= data.ranges
        self.laser = data
        self.angle_increment = data.angle_increment
        #print(self.angle_increment)
        #print(l)
        #print("0",data.ranges[0])
        #print("100",data.ranges[100])
        #print("200",data.ranges[200])
        #print("300",data.ranges[300])

    def red_callback(self,data):
        print("red")
        red_pt = PointStamped()
        lidar_red_pt = PointStamped()
        lidar_red_pt_ = PointStamped()
        red_pt.header.seq = self.laser.header.seq
        red_pt.header.stamp = self.laser.header.stamp
        red_pt.header.frame_id = "camera_depth_optical_frame"
        l =len(data.data)
        listener = tf.TransformListener()
        listener2 = tf.TransformListener()
        i=0
        #rint("red")
        while i<l:
            #print(i)
            red_pt.point.x = data.data[i]
            red_pt.point.y = data.data[i+1]
            red_pt.point.z = data.data[i+2]
            try:
                lidar_red_pt = listener.transformPoint("rplidar_frame",red_pt)
                #print(lidar_red_pt)
            except:
                pass
            #print(lidar_red_pt)
            angle =math.atan2(lidar_red_pt.point.y,lidar_red_pt.point.x)
            angle_increment = self.angle_increment
            distance = 0
            #print(angle)
            if (0 < angle) and (angle<math.pi/2.0):
                #pos_ = angle/angle_increment
                pos = int(round(angle/angle_increment,0))
                #print("pos",pos)
                laser_data = self.laser
                distance = laser_data.ranges[pos]
                #print("distance",distance)
            elif (angle<0) and (abs(angle)<math.pi/2.0):
                pos = 399 - int(round(abs(angle)/angle_increment,0))
                laser_data = self.laser
                distance = laser_data.ranges[pos]
                #print("distance",distance)
            else:
                print("else")
            lidar_red_pt.header.stamp = rospy.Time(0)
            lidar_red_pt.header.frame_id = "rplidar_frame"

            try:
                if (red_pt.point.z == 1.0):
                    lidar_red_pt.point.x *= distance
                    lidar_red_pt.point.y *= distance
                    lidar_red_pt.point.z *= distance

                else:
                    lidar_red_pt.point.x = (lidar_red_pt.point.x/lidar_red_pt.point.x)*distance
                    lidar_red_pt.point.y = (lidar_red_pt.point.y/lidar_red_pt.point.x)*distance
                    lidar_red_pt.point.z = (lidar_red_pt.point.z/lidar_red_pt.point.x)*distance
            except:
                pass
            #print(lidar_red_pt)
            #lidar_red_pt_=listener.transformPoint("base_link", lidar_red_pt)
            #print(lidar_red_pt_)
            try:
                lidar_red_pt_=listener.waitForTransform("rplidar_frame", "map", rospy.Time(0), rospy.Duration(5.0))
                lidar_red_pt_=listener.transformPoint("map", lidar_red_pt)
                #print(lidar_red_pt_)
                """marker_red = Marker()
                marker_red.header = lidar_red_pt_.header
                marker_red.ns = "danger_zones"
                marker_red.id =  self.j
                marker_red.type = Marker.CUBE
                #marker_red.action = 2
                marker_red.pose.position.x = round(lidar_red_pt_.point.x,3)
                marker_red.pose.position.y = round(lidar_red_pt_.point.y,3)
                marker_red.pose.position.z = round(lidar_red_pt_.point.z,3)
                marker_red.pose.orientation.x = 0.0
                marker_red.pose.orientation.y = 0.0
                marker_red.pose.orientation.z = 0.0
                marker_red.pose.orientation.w = 1.0
                marker_red.scale.x = 0.1
                marker_red.scale.y = 0.2
                marker_red.scale.z = 0.2
                marker_red.color.r = 255
                marker_red.color.g = 0
                marker_red.color.b = 0
                marker_red.color.a = 1
                print(marker_red)"""
                #print(marker_red)
                try:
                    #print(marker_red)
                    #marker_red.lifetime = rospy.Duration();
                    self.marker_red_pub.publish(lidar_red_pt_)
                    #rospy.sleep(1)
                    #print(marker_red)
                except:
                    print("3")
                    pass
                #print(lidar_red_pt_)

            except:
                pass
            i+=3
            self.j+=1



    def green_callback(self,data):
        #rint("blue")
        green_pt = PointStamped()
        lidar_green_pt = PointStamped()
        lidar_green_pt_ = PointStamped()
        green_pt.header.seq = self.laser.header.seq
        green_pt.header.stamp = self.laser.header.stamp
        green_pt.header.frame_id = "camera_depth_optical_frame"
        l =len(data.data)
        listener = tf.TransformListener()
        listener2 = tf.TransformListener()
        i=0
        while i<l:
            green_pt.point.x = data.data[i]
            green_pt.point.y = data.data[i+1]
            green_pt.point.z = data.data[i+2]
            try:
                lidar_green_pt = listener.transformPoint("rplidar_frame",green_pt)
                #print(lidar_green_pt)
            except:
                print("1")
                pass
            angle =math.atan2(lidar_green_pt.point.y,lidar_green_pt.point.x)
            #print("angle",angle)
            angle_increment = self.angle_increment
            distance = 0
            #print(angle)
            if (0 <= angle) and (angle<=math.pi/2.0):
                #pos_ = angle/angle_increment
                #print(angle)
                #print(angle_increment)
                pos = int(round(angle/angle_increment,0))
                #print("pos",pos)
                laser_data = self.laser
                distance = laser_data.ranges[pos]
                #print("distance1",distance)
                #print("pos1",pos)
                #print("distance",distance)
            elif (angle<0) and (abs(angle)<=math.pi/2.0):
                pos = 399 - int(round(abs(angle)/angle_increment,0))
                laser_data = self.laser
                distance = laser_data.ranges[pos]
                #print("distance2",distance)
                #print("pos2",pos)
            else:
                print("else")
            lidar_green_pt.header.stamp = rospy.Time(0)
            lidar_green_pt.header.frame_id = "rplidar_frame"
            try:
                if (green_pt.point.x == 1.0):
                    lidar_green_pt.point.x *= distance
                    lidar_green_pt.point.y *= distance
                    lidar_green_pt.point.z *= distance
                    #print("point_x=1",lidar_green_pt.point.x )

                else:
                    lidar_green_pt.point.x = (lidar_green_pt.point.x/lidar_green_pt.point.x)*distance
                    lidar_green_pt.point.y = (lidar_green_pt.point.y/lidar_green_pt.point.x)*distance
                    lidar_green_pt.point.z = (lidar_green_pt.point.z/lidar_green_pt.point.x)*distance
                    #print("point_x!=1",lidar_green_pt.point.x)
            except:
                print("2")
                pass
            try:
                lidar_green_pt_=listener.waitForTransform("rplidar_frame", "map", rospy.Time(0), rospy.Duration(5.0))
                lidar_green_pt_=listener.transformPoint("map", lidar_green_pt)
                #print("3")
                """marker_green = Marker()
                marker_green.header = lidar_green_pt_.header
                marker_green.ns = "person"
                marker_green.id = i/3.0
                marker_green.type = Marker.CUBE
                #marker_green.action = 2
                marker_green.pose.position.x = round(lidar_green_pt_.point.x,3)
                marker_green.pose.position.y = round(lidar_green_pt_.point.y,3)
                marker_green.pose.position.z = round(lidar_green_pt_.point.z,3)
                marker_green.pose.orientation.x = 0.0
                marker_green.pose.orientation.y = 0.0
                marker_green.pose.orientation.z = 0.0
                marker_green.pose.orientation.w = 1.0
                marker_green.scale.x = 0.2
                marker_green.scale.y = 0.2
                marker_green.scale.z = 0.2
                marker_green.color.r = 0
                marker_green.color.g = 0
                marker_green.color.b = 255
                marker_green.color.a = 1"""
                #print(marker_green)
                try:
                    self.marker_green_pub.publish(lidar_green_pt_)
                    #marker_green.lifetime = rospy.Duration();
                    #print(marker_green)
                except:
                    print("3")
                    pass
            except:
                print("4")
                pass
            #print(lidar_green_pt_)
            i+=3
            self.j+=1
