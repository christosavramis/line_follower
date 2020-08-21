#!/usr/bin/env python

import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from std_msgs.msg import String

class Follower:
  def __init__(self):
    self.bridge = cv_bridge.CvBridge()
    self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback)
    self.cmd_vel_pub = rospy.Publisher('direction', String, queue_size=1)
    
  def colorthresh(self, image):
    # Detect all objects within the HSV range
    # image = cv2.GaussianBlur(image, (3,3), 0.1, 0.1)
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower_yellow = numpy.array([ 20, 100, 100])
    upper_yellow = numpy.array([ 30, 255, 255])
    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
    
    h,w,_ = image.shape
    mask[0:w, 0:int(0.8*h)] = 0 # (0, 0, w, 0.8*h)
    # mask[0:w, int(0.6*h):h] = 0 # (0, 0.2*h, w, h)
    # mask[int(0.7*w):int(0.3*w), 0:h] = 0 # (0.7*w, 0, 0.3*w, h)
    # mask[0:int(0.3*w), 0:h] = 0 # (0, 0, 0.3*w, h)
    
    # Perform centroid detection of line
    M = cv2.moments(mask)
    cx = 0.0
    if M['m00'] > 0:
      cx = int(M['m10']/M['m00'])
      # cy = int(M['m01']/M['m00'])
    
    tol = 25
    count = cv2.countNonZero(mask)

    if cx < w/2-tol:
      message = 'FL'
    elif cx > w/2+tol:
      message = 'FR'
    elif count == 0:
      message = 'RT'
    else:
      message = 'F'

    cv2.imshow("mask",mask)
    cv2.imshow("output", image)

    return message
    
  def image_callback(self, msg):
    image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
    message = self.colorthresh(image)
    self.cmd_vel_pub.publish(message)
    cv2.waitKey(3)

if __name__ == '__main__':
  try:
    rospy.init_node('line_follower')
    follower = Follower()
    rospy.spin()
  except rospy.ROSInterruptException: pass
