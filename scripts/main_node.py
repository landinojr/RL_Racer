#!/usr/bin/env python
import rospy
import sys
#import regex
import re
#imprt action library
from std_msgs.msg import String

rospy.init_node('main_node')
#wait for service to be up and running
rospy.wait_for_service('face_recog_rev')
#set proxy call
face_find_instruction = rospy.ServiceProxy('face_recog_rev', face_recog_service)



rate = rospy.Rate(2)
#while the robo is not shut down
while not rospy.is_shutdown():
    # Gets the persons name from the uers
    user_msg = raw_input("Who would you like to find: ")
    print 'we are going to scan your face please make sure your face is visable on sceen'
    print 'count down has began'

    # Tells the person to get in the frame in 5 secsounds
    for i in range(5,0):
        print "scan will begin in", i
    print 'scanning'
    # Service Proxy call
    face_name = move_me_instrcution(str(user_msg))


    # If no faces
    if face_name == None:
        print 'we can not see a face in the frame'
    # If the person is unknown
    if face_name == 'Unknown':
        print 'We can see a face, but you are not in my database'
    # If the person is known
    else:
        print 'Hey', face_name,'hows it going'




    rate.sleep()
