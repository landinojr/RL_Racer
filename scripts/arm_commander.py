#!/usr/bin/env python
#Publishes Commands to ROS ARM in format below

import rospy
from std_msgs.msg import String

face = 'cant_find_face'
done=False

def callback(name):
    global done

    if name != face and not done:
        pub.publish("MANIP 0 \n")
        done=True

    # else:
    #     pub.publish("MANIP 1 \n")
    global face
    face = name
    print face






# Make this into a ROS node.
rospy.init_node('topic_publisher')

# Prepare to publish topic `counter`
pub = rospy.Publisher('armcommand', String, queue_size=10)
color_sub = rospy.Subscriber("face_recognition", String, callback)

# sleep at rate of loops per second
rate = rospy.Rate(.5)
commands= ["MANIP 1 \n","ARM 1 330 1 20 \n","MANIP 0 \n","MANIP 2 \n","ARM 1 1 60 20 \n"]
index=0
pub.publish("MANIP 1 \n")
# loop until ^c
rospy.spin()
