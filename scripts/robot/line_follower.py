#!/usr/bin/env python

import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from std_msgs.msg import String

class Follower:
	
	def __init__(self):
		self.bridge = cv_bridge.CvBridge()
		self.image_sub = rospy.Subscriber('usb_cam/image_raw', Image, self.image_callback)
		self.cmd_vel_pub = rospy.Publisher('direction', String, queue_size=1)
		self.last_message = ''

	def line_detection(self, image):
		h,w,_ = image.shape
		image_det = image[int(0.8*h):h,0:w]                             # Reduce image size 
		image_det_gray = cv2.cvtColor(image_det, cv2.COLOR_BGR2GRAY )   # Reduce image channels from BGR to Gray
		_,th1 = cv2.threshold(image_det_gray,50,255,cv2.THRESH_BINARY)  # Image filter making only the tape visible
		th1 = cv2.subtract(255, th1)                                    # Reverse the filter

		# Find the center of the black  tape
		cx,cy = 0,0
		m = cv2.moments(th1)
		if m['m00'] > 0:
			cx = int(m['m10']/m['m00'])
			cy = int(m['m01']/m['m00'])

		# Detect if there filter is empty => there is no black tape detected
		count = cv2.countNonZero(th1)
		forward_tolerance = 90
		rotational_tolerance = 55

		# Draw center line
		cv2.rectangle(image_det, (int(w/2)+forward_tolerance,h), (int(w/2)-forward_tolerance,0), (0, 0, 255), 5) 
		cv2.rectangle(image_det, (int(w/2)+rotational_tolerance,h), (int(w/2)-rotational_tolerance,0), (0, 255, 0), 5)

		# Draw center of the black tape
		cv2.circle(image_det,(cx,cy), 5,(255,0,255),-1)
		cv2.imshow("image_det",image_det)
		cv2.imshow("th1",th1)
				
		if cx > w/2 + forward_tolerance:		# Righthand deviation
			message = 'R'							# Rotate clockwise
		elif cx < w/2 - forward_tolerance:		# Lefthand deviation
			message = 'L'							# Rotate counter-clockwise
		elif cx > w/2 + rotational_tolerance:	# Slight rigthand deviation
			message = 'FR'							# Lean right while moving forward
		elif cx < w/2 - rotational_tolerance:	# Slight lefthand deviation
			message = 'FL'							# Lean left while moving forward
		elif count == 0:						# Line not found
			message = 'NF'							# Smart recovery
		else:									# Line Centered
			message = 'F'							# Move forward

		return message

	def image_callback(self, msg):
		image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
		message = self.line_detection(image)
		if message != self.last_message:
			if message == 'NF' and ( self.last_message == 'R' or self.last_message == 'FR'):
				message = 'R'
			elif message == 'NF' and ( self.last_message == 'L' or self.last_message == 'FL'):
				message = 'L'
			elif message == 'NF' and self.last_message == 'F':
				message = 'B'
			elif message == 'NF':
				message = 'F'
			self.cmd_vel_pub.publish(message)
			self.last_message = message
		cv2.waitKey(1)

if __name__ == '__main__':
	try:
		rospy.init_node('line_follower')
		follower = Follower()
		rospy.spin()
	except rospy.ROSInterruptException: pass
