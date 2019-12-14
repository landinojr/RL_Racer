#!/usr/bin/env python
#importing Python labraies
import rospy
import numpy as np
from std_msgs.msg import String
from geometry_msgs.msg import Twist

# create global variables
global state
global color


state =  1
color = 'blue'

def callback(msg):
    color = msg
    print color


# cretes a ROS node
rospy.init_node('red_light_gren_light')
# Subscribe to color_detector Node
color_sub = rospy.Subscriber("/color_detector", String, callback)
# Publisher for command vel
cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
# Sets rate
rate = rospy.Rate(2)
# state machn=ine that deterines what to do
while not rospy.is_shutdown():
    move_me = Twist()
    #  if red stop
    if color == 'red':
        # sets state to 0
        state = 0
    # If blue go
    elif color == 'blue':
        state = 1

    if state == 1:
        move_me.linear.x = .3
        cmd_vel_pub.publish(move_me)
    elif state ==0:
        move_me.linear.x = 0
        cmd_vel_pub.publish(move_me)
    # Print the whats going on
    print 'Twist = ',move_me.linear, ' color =' , color , ' state =' , state
    rate.sleep()
rospy.spin()
