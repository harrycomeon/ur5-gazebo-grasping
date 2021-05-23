#!/usr/bin/env python
################################################################################
#
# Introduction
##############
# This script generates three images RGB,HSV,MASK
# 
from __future__ import print_function
import roslib
#roslib.load_manifest('my_package')
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
class image_converter:
  def __init__(self):
    self.image_pub = rospy.Publisher("/Kinect_V2/ir/image_raw_converter",Image,queue_size=10)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/kinect_V2/rgb/image_raw",Image,self.callback)
  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    (rows,cols,channels) = cv_image.shape
    cv2.imshow("Image window", cv_image)
    cv2.waitKey(3)

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError as e:
      print(e)

class RGB_to_HSV_to_MASK:
  def __init__(self):
      self.bridge = CvBridge()
      self.image_sub = rospy.Subscriber("/kinect_V2/rgb/image_raw",Image,self.callback)
  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
       print(e)

    # Convert from RGB to HSV
    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    lower_wood = np.array([10,43,46])
    upper_wood = np.array([18,255,255])
    # Threshold the HSV image to get only white colors
    mask = cv2.inRange(hsv, lower_wood, upper_wood)
    
    (rows,cols,channels) = cv_image.shape
    # Calculate centroid of the blob of binary image using ImageMoments
    m = cv2.moments(mask, False)
    try:
        cx, cy = m['m10']/m['m00'], m['m01']/m['m00']
    except ZeroDivisionError:
        cy, cx = rows/2, cols/2    
    # Draw the centroid in the resultut image
    # cv2.circle(img, center, radius, color[, thickness[, lineType[, shift]]]) 
    cv2.circle(mask,(int(cx), int(cy)), 10,(0,0,255),-1)
    print(cx,cy)
    cv2.imshow("HSV", hsv)
    cv2.imshow("MASK", mask)

    cv2.waitKey(1)

def main(args):
  ic = image_converter()
  mask_image = RGB_to_HSV_to_MASK()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()
if __name__ == '__main__':
    main(sys.argv)