#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String


def callback(data):
    # Starts a new node

    vel_msg = Twist()
    
    if data.data == 'F':
	vel_msg.linear.x = 0.1
	vel_msg.angular.z = 0
    elif data.data == 'FR':
	vel_msg.linear.x = 0.1
	vel_msg.angular.z = -1
    elif data.data == 'FL':
	vel_msg.linear.x = 0.1
	vel_msg.angular.z = 1
    else :
	vel_msg.linear.x = 0
	vel_msg.angular.z = 0

    velocity_publisher.publish(vel_msg)


def listener():
    rospy.init_node('simulation_move', anonymous=True)
    rospy.Subscriber("direction",String, callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        #Testing our function
	velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        listener()
    except rospy.ROSInterruptException: pass
