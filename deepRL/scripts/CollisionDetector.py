#! /usr/bin/env python2
import rospy
from rlracer.msg import AveragedScan,NewEpisode
from std_msgs.msg import Bool
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math

rospy.init_node('CollisionDetector')
collision_pub = rospy.Publisher('collision_detected',Bool,queue_size=10)
min_distance = 1
curLinearVelocity = 0
oldX = 0
oldY = 0
linearX = 0
errorThresh = .07
modelStateV = 0
updateTime = .01
badCounter = 0
while(rospy.Time.now().to_sec() == 0):
    continue
oldTime = rospy.Time.now().to_sec()

def new_episode_callback(msg):
    global badCounter
    if (msg.is_new_episode):
        badCounter = 0

def averaged_scan_callback(msg):
    global min_distance
    min_distance = min(msg.averaged_scan)
    #print(str(min_distance))

def model_state_callback(msg):
    global oldX
    global oldY
    global oldTime
    global modelStateV
    global updateTime
    now = rospy.Time.now().to_sec()
    if ((now - oldTime) < updateTime):
        return
    for ind,name in enumerate(msg.name):
        curX = msg.pose[ind].position.x
        curY = msg.pose[ind].position.y
    now = rospy.Time.now().to_sec()
    dt = now - oldTime
    modelStateV = math.sqrt((oldX - curX)**2 + (oldY - curY)**2)/dt
    oldTime = now
    oldX = curX
    oldY = curY

def cmd_vel_callback(msg):
    global linearX
    linearX = abs(msg.linear.x)

model_state_sub = rospy.Subscriber('/gazebo/model_states',ModelStates,model_state_callback)
averaged_scan_sub = rospy.Subscriber('/averaged_scan', AveragedScan, averaged_scan_callback)
cmd_vel_sub = rospy.Subscriber('/cmd_vel',Twist,cmd_vel_callback)
new_episode_sub = rospy.Subscriber('/is_new_episode',NewEpisode,new_episode_callback)
rate = rospy.Rate(20)
badMax = 10
concurrentCrashes = 0
while not rospy.is_shutdown():
    isCollision = Bool()
    """
    if (abs(linearX - modelStateV) > errorThresh):
        print('error thresh exceeded | linearX = ' + str(linearX) + ' | modelV = ' + str(modelStateV))
        print('dV = ' + str(abs(linearX - modelStateV)))
        if (min_distance < .024):
            print("Collision detected")
            isCollision.data = True
        else:
            isCollision.data = False
    else:
        isCollision.data = False
    
    if ((linearX > .001 and modelStateV < .001) and (min_distance < .024)):
        isCollision.data  = True
    else:
        isCollision.data = False
    if (abs(linearX - modelStateV) > errorThresh):
        badCounter += 1
        print(str(linearX) + "    " + str(modelStateV))
        print("Error thresh exceeded: " + str(badCounter))
        if (badCounter > badMax):
            isCollision.data = True
            badCounter = 0
        else:
            isCollision.data = False
    """
    #print(str(min_distance))
    if ((min_distance < .024 and abs(linearX - modelStateV) > errorThresh) and modelStateV < .05):
        concurrentCrashes += 1
        if (concurrentCrashes >= 1):
            isCollision.data = True
    else:
        isCollision.data = False
        concurrentCrashes = 0

    #print("CollisionDetector: " + str(isCollision.data))
    collision_pub.publish(isCollision)
    rate.sleep()
rospy.spin()
