#!/usr/bin/env python
import rospy
import numpy as np
import cv2

from sensor_msgs.msg import Image, CameraInfo,CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String


# Method that conversts the ROS image to Open_Cv format
def covert_from_ros(img):
    # Try castch just in case it conversion deos not go smoothly
    try:
        # creates the bridge object
        bridge = CvBridge()
        # Creates the cab image
        cv_image = bridge.compressed_imgmsg_to_cv2(img, "bgr8")
        # return cv_image
        return cv_image
    except CvBridgeError as e:
        print(e)

def callback(data):
    # convert the ROS image
    cap = covert_from_ros(data)


    hsv = cv2.cvtColor(cap, cv2.COLOR_BGR2HSV)
    # State the color ranges
    lower_blue = np.array([100,50,50])
    upper_blue = np.array([130,255,255])

    low_red = np.array([161, 155, 84])
    high_red = np.array([179, 255, 255])

    low_green = np.array([25, 52, 72])
    high_green = np.array([102, 255, 255])

    # Sets the masks for color detection and computer if the color
    # exist in the fram
    mask = cv2.inRange (hsv, lower_blue, upper_blue)
    bluecnts = cv2.findContours(mask.copy(),
                          cv2.RETR_EXTERNAL,
                          cv2.CHAIN_APPROX_SIMPLE)[-2]

    mask_red = cv2.inRange (hsv, low_red, high_red)
    redcnts = cv2.findContours(mask_red.copy(),
                          cv2.RETR_EXTERNAL,
                          cv2.CHAIN_APPROX_SIMPLE)[-2]
    green_mask = cv2.inRange(hsv, low_green, high_green)
    green_cnts = cv2.findContours(green_mask.copy(),
                          cv2.RETR_EXTERNAL,
                          cv2.CHAIN_APPROX_SIMPLE)[-2]

    print len(redcnts) > 0
    print len(bluecnts) > 0

    # based on the colors on the screen publisg the color
    if len(redcnts)>0:
        red_area = max(redcnts, key=cv2.contourArea)
        (xg,yg,wg,hg) = cv2.boundingRect(red_area)
        pub.publish('red')
        print 'sending red'
    elif len(bluecnts)>0:
        blue_area = max(bluecnts, key=cv2.contourArea)
        (xg,yg,wg,hg) = cv2.boundingRect(blue_area)
        pub.publish('blue')
    else:
        pub.publish('no_col')
    rate.sleep()


# Create ROS node
rospy.init_node('racecar_signal')

pub = rospy.Publisher('/color_detector', String, queue_size=1)
# subscribes to ROS images
image_sub = rospy.Subscriber("raspicam_node/image/compressed", CompressedImage, callback)

rate = rospy.Rate(.75)
rospy.spin()
