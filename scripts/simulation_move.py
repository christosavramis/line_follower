#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String


def callback(data):
  vel_msg = Twist()
  
  if data.data == 'F':
    # Going straight
    vel_msg.linear.x = 0.25
    vel_msg.angular.z = 0
  elif data.data == 'FR':
    # Turning right
    vel_msg.linear.x = 0.1
    vel_msg.angular.z = -0.15
  elif data.data == 'FL':
    # Turning left
    vel_msg.linear.x = 0.1
    vel_msg.angular.z = 0.15
  else :
    # Searching
    vel_msg.linear.x = 0
    vel_msg.angular.z = 0.30

  velocity_publisher.publish(vel_msg)

if __name__ == '__main__':
  try:
    rospy.init_node('simulation_move', anonymous=True)
    velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber("direction",String, callback)
    rospy.spin()
  except rospy.ROSInterruptException: pass
