#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from center_contour import CenterContour
from cv_bridge import CvBridge, CvBridgeError
from classes import image_converter,lidar_converter
from std_msgs.msg import Empty
import sys
import cv2

"""def call_vision(req):
    lc = lidar_converter()
    print (req)
    return Empty()"""

def main():
    rospy.init_node('camera_rplidar', anonymous=True)
    #try:

    #s = rospy.Service('call_vision', Empty, call_vision)
    #centers = {}
    ic = image_converter()
    lc = lidar_converter()
    #od = obtain_direction()
    #print(ic.centers)
    #except:
    #    print("Error obtaining marker's center")
    try:
      rate = rospy.Rate(2) # 10hz
      rospy.spin()
    except KeyboardInterrupt:
      print("Shutting down")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
