#!/usr/bin/env python

import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from std_msgs.msg import String

class Follower:
	
	def __init__(self):
		self.bridge = cv_bridge.CvBridge()
		self.image_sub = rospy.Subscriber('usb_cam/image_raw', Image, self.image_callback)
		self.cmd_vel_pub = rospy.Publisher('direction', String, queue_size=1)
		self.message = ''

	def line_detection(self, image):
		h,w,_ = image.shape
		image_det = image[int(0.8*h):h,0:w]                             #reduce image size 
		image_det_gray = cv2.cvtColor(image_det, cv2.COLOR_BGR2GRAY )   #reduce image channels from BGR to Gray
		_,th1 = cv2.threshold(image_det_gray,50,255,cv2.THRESH_BINARY)  #image filter making only the tape visible
		th1 = cv2.subtract(255, th1)                                    #reverse the filter

		#find the center of the black  tape
		cx,cy = 0,0
		m = cv2.moments(th1)
		if m['m00'] > 0:
			cx = int(m['m10']/m['m00'])
			cy = int(m['m01']/m['m00'])

		#detect if there filter is empty => there is no black tape detected
		count = cv2.countNonZero(th1)
		tolerance = 25
		
		if cx < w/2 - tolerance and self.message !='FL':     # o||
			self.message = 'FL'
		elif cx > w/2 + tolerance and self.message !='FR':   # ||o
			self.message = 'FR'
		elif count == 0 and self.message !='RL':             # ||
			self.message = 'RT'
		elif self.message !='F':                             # |o|
			self.message = 'F'
			
		#draw center line
		cv2.rectangle(image_det, (int(w/2)+tolerance,h), (int(w/2)-tolerance,0), (255, 0, 0) , 5) 

		#draw center of the black tape
		cv2.circle(image_det,(cx,cy), 5,(0,0,255),-1)
		cv2.imshow("image_det",image_det)
		cv2.imshow("th1",th1)
				
		return self.message

	def image_callback(self, msg):
		image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
		message = self.line_detection(image)
		self.cmd_vel_pub.publish(message)
		cv2.waitKey(1)

if __name__ == '__main__':
	try:
		rospy.init_node('line_follower')
		follower = Follower()
		rospy.spin()
	except rospy.ROSInterruptException: pass