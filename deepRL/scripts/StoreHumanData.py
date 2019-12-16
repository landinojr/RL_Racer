#! /usr/bin/env python2
import rospy
from myRLclasses.ddpg import DDPGagent
from rlracer.msg import AveragedScan
from std_msgs.msg import Bool,Float64
from geometry_msgs.msg import Twist
import numpy as np
import pickle
import os.path
from os import path

rospy.init_node('StoreHumanData')
agentFile = 'agent7.pkl'
if (path.exists('/home/ben/catkin_ws/src/rlracer/' + agentFile)):
    print(agentFile + ' file exists. loading...')
    with open('/home/ben/catkin_ws/src/rlracer/' + agentFile,'rb') as f:
        agent = pickle.load(f)
else:
    agent = DDPGagent(360)
    with open('/home/ben/catkin_ws/src/rlracer/' + agentFile,'w') as f:
        pickle.dump(agent,f)
    print(agentFile + ' file created')

curReward = 0
oldState = []
curState = []
isCollision = False
curReward = 0
curAction = np.asarray([0,0])
def averaged_scan_callback(msg):
    global curState
    curState = np.asarray(msg.averaged_scan)

def collision_callback(msg):
    global isCollision
    isCollision = msg.data

def current_reward_callback(msg):
    global curReward
    global isCollision
    curReward = msg.data - isCollision

def cmd_vel_callback(msg):
    global curAction
    a = [msg.linear.x, msg.angular.z]
    curAction = np.asarray(a)

averaged_scan_sub = rospy.Subscriber('averaged_scan',AveragedScan,averaged_scan_callback)
collision_sub = rospy.Subscriber('/collision_detected',Bool,collision_callback)
current_reward_sub = rospy.Subscriber('current_reward',Float64,current_reward_callback)
cmd_vel_sub = rospy.Subscriber('/cmd_vel',Twist,cmd_vel_callback)

while(curState == []):
    continue
    #print("Waiting for averaged scan")
rate = rospy.Rate(10)
t = 1
batch_size = 1024
while not rospy.is_shutdown():
    if (t > 10):
        reward = curReward
        agent.memory.push(oldState,action,reward,curState)
    oldState = np.asarray(curState)
    action = curAction
    if (len(agent.memory) > batch_size):
        agent.update(batch_size)
        if ((t%5000) == 0):
            with open('/home/ben/catkin_ws/src/rlracer/' + agentFile,'w') as f:
                pickle.dump(agent,f)
            print("AGENT UPDATED AND SAVED")
    print(str(t) + ": reward = " + str(curReward))
    print("isCollision = " + str(isCollision))
    t += 1
    rate.sleep()
