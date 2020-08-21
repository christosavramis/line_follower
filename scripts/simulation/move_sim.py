#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String


def callback(data):
  vel_msg = Twist()
#  rospy.loginfo(data.data)
  
  if data.data == 'F':
    # Going forward
    vel_msg.linear.x = 0.5
    vel_msg.angular.z = 0
  elif data.data == 'FR':
    # Leaning right
    vel_msg.linear.x = 0.35
    vel_msg.angular.z = -0.15
  elif data.data == 'R':
    # Turning right
    vel_msg.linear.x = 0
    vel_msg.angular.z = -0.3
  elif data.data == 'FL':
    # Leaning left
    vel_msg.linear.x = 0.35
    vel_msg.angular.z = 0.15
  elif data.data == 'L':
    # Turning left
    vel_msg.linear.x = 0
    vel_msg.angular.z = 0.3
  elif data.data == 'B':
    # Going backward
    vel_msg.linear.x = -0.35
    vel_msg.angular.z = 0
  else:
    # Stopping
    vel_msg.linear.x = 0
    vel_msg.angular.z = 0

  velocity_publisher.publish(vel_msg)

if __name__ == '__main__':
  try:
    rospy.init_node('simulation_move', anonymous=True)
    velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    rospy.Subscriber("direction", String, callback)
    rospy.spin()
  except rospy.ROSInterruptException: pass
