#!/usr/bin/env python

#importing Python labraies
import face_recognition
import cv2
import numpy as np
import rospy
#importing ROS labraies and messesge/data type
from sensor_msgs.msg import Image, CameraInfo,CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String

# Creates a global variable
global frame

# Loading facesfrom jpg files to store them
# find the faces in the pictures uploaded and encode the faces
luis_image = face_recognition.load_image_file("luis.jpg")
luis_face_encoding = face_recognition.face_encodings(luis_image)[0]

pito_image = face_recognition.load_image_file("pito.jpg")
pito_face_encoding = face_recognition.face_encodings(pito_image)[0]

# Creating list and adding facial ecncodings
known_face_encodings = [
    luis_face_encoding,
    pito_face_encoding
]
# Create a list of name for the paper
known_face_names = [
    "luis andino",
    "pito",
]
face_locations = []
face_encodings = []
face_names = []


def callback(msg):
    try:
        bridge = CvBridge()
        cv_image = bridge.compressed_imgmsg_to_cv2(msg, "rgb8")
        cv_image = cv2.rotate(cv_image, cv2.ROTATE_180)
        frame = cv_image
    except CvBridgeError as e:
        print(e)
# service call back to find faces
def find_face(name):

    if frame is None:
        print 'no frame to publish'
    # condences the frame to make it smaller
    small_frame = cv2.resize(frame, (0, 0), fx=0.25, fy=0.25)

    # Converts to RGB
    rgb_small_frame = small_frame[:, :, ::-1]


    # Find all the faces and face encodings in the current frame of video
    face_locations = face_recognition.face_locations(rgb_small_frame)
    face_encodings = face_recognition.face_encodings(rgb_small_frame, face_locations)
    # List of names
    face_names = []
    # Iterates through all faces that it found in the face encodings
    for face_encoding in face_encodings:
        # Comapares the facce to known faces
        matches = face_recognition.compare_faces(known_face_encodings, face_encoding)
        # Sets a name variable to unknwn
        name = "Unknown"
        # gets the face distances
        face_distances = face_recognition.face_distance(known_face_encodings, face_encoding)
        # Gets the index of the name of the person
        best_match_index = np.argmin(face_distances)
        # Sets the name variable to the person that it found
        if matches[best_match_index]:
            name = known_face_names[best_match_index]
            print(name)
        face_names.append(name)
    return name




# Creats the node
rospy.init_node('face_recog_service')
# Creates the service
service = rospy.Service("face_recog_rev", face_recog_service, find_face)
# subscribes to ROS images
image_sub = rospy.Subscriber("raspicam_node/image/compressed", CompressedImage, callback)

# Quits when ^c is pressed in terminal 
rospy.spin()
