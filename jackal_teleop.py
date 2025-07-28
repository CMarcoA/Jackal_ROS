#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

latest_twist = Twist()

def joy_callback(data):
    global latest_twist

    # Map joystick to Twist
    latest_twist.linear.x = data.axes[1] * 0.5
    latest_twist.angular.z = data.axes[3] * 1.0

    rospy.loginfo("Received /joy → linear.x: %.2f, angular.z: %.2f",
                  latest_twist.linear.x, latest_twist.angular.z)

def publish_twist(event):
    pub.publish(latest_twist)

def main():
    global pub
    rospy.init_node('xbox_teleop', anonymous=True)

    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/joy', Joy, joy_callback)

    # Timer: call publish_twist every 0.05s (~20Hz)
    rospy.Timer(rospy.Duration(0.05), publish_twist)

    rospy.loginfo("xbox_teleop node started. Waiting for joystick input...")
    rospy.spin()

if __name__ == '__main__':
    main()
