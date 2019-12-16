#! /usr/bin/env python2
import rospy
#import myRLclasses.ddpg
from myRLclasses.ddpg import DDPGagent
#import myRLclasses.models
#import myRLclasses.utils
from myRLclasses.utils import OUNoise
from gazebo_msgs.msg import ModelStates,ModelState
from gazebo_msgs.srv import SetModelState
from geometry_msgs.msg import Pose,Twist
from std_msgs.msg import Float64,Bool
from tf.transformations import quaternion_from_euler
from rlracer.msg import AveragedScan,NewEpisode
import numpy as np
import pickle
import os.path
from os import path
import matplotlib.pyplot as plt

xs = [-3.000, -2.674, -2.453, -2.172, -2.018, -1.772, -1.307, -.933, -.499, .003, .465, .890, 1.354, 1.756, 2.506, 2.946, 3.293, 3.030, 2.630, 1.785, .889, -.110, -.773, -1.006, -1.009, -1.159, -1.896, -2.661, -2.891, -3.092, -3.191]

ys = [1.500, 1.500, 1.500, 1.550, 1.601, 1.735, 1.739, 1.447, 1.138, .849, .719, .729, .743, .959,1.027, .834, .348, -.485, -.599, -.524, -.486, -.485, -.619, -1.008, -1.663, -2.077, -2.040, -1.626, -.912, -.209, .644]
rospy.init_node('RLMaster_multiStart')
rospy.wait_for_service('/gazebo/set_model_state')
reset_service = rospy.ServiceProxy('/gazebo/set_model_state',SetModelState)
reset_pub = rospy.Publisher('/gazebo/set_model_states',ModelStates,queue_size=10)
cmd_vel_pub = rospy.Publisher('cmd_vel',Twist,queue_size=10)
new_episode_pub = rospy.Publisher('/is_new_episode',NewEpisode,queue_size=10)
curModelState = ModelStates()
startX = -3.000
startY = 1.5
curReward = 0
isCollision = False
curState = []
oldState = []
totalStep = 1
action_mins = [-.22, -.5]
action_maxs = [.22, .5]
def averaged_scan_callback(msg):
    global curState
    curState = np.asarray(msg.averaged_scan)

def model_state_callback(msg):
    global curModelState
    curModelState = msg

def collision_callback(msg):
    global isCollision
    isCollision = msg.data
    #print("isCollision: " + str(isCollision))

def current_reward_callback(msg):
    global curReward
    curReward = msg.data

def reset_model_state(startPoint):
    global curModelState
    global isCollision
    global xs
    global ys
    print("reset_model_state")
    if (startPoint >= (len(xs)-1)):
        dx = xs[0] - xs[-1]
        dy = ys[0] - ys[-1]
    else:
        dx = xs[startPoint+1] - xs[startPoint]
        dy = ys[startPoint+1] - ys[startPoint]
    if (dx > 0):
        startAngle = np.arctan((dy/dx))
    else:
        startAngle = np.pi - np.arctan((dy/-dx))
    startQuaternion = quaternion_from_euler(0.0,0.0,startAngle)
    resetModelState = ModelState()
    resetModelState.model_name = "turtlebot3_burger"
    resetModelState.pose.position.x = xs[startPoint]
    resetModelState.pose.position.y = ys[startPoint]
    resetModelState.pose.position.z = 0
    resetModelState.pose.orientation.x = startQuaternion[0]
    resetModelState.pose.orientation.y = startQuaternion[1]
    resetModelState.pose.orientation.z = startQuaternion[2]
    resetModelState.pose.orientation.w = startQuaternion[3]
    #resetModelState.pose.orientation = startQuaternion
    resetTwist = Twist()
    resetTwist.linear.x = 0
    resetTwist.linear.y = 0
    resetTwist.linear.z = 0
    resetTwist.angular.x = 0
    resetTwist.angular.y = 0
    resetTwist.angular.z = 0
    resetModelState.twist = resetTwist
    resp = reset_service(resetModelState)
    isCollision = False

def run_episode(agent,noise,startPoint):
    global isCollision
    global curState
    global oldState
    global allEpisodeRewards
    global curReward
    global totalStep
    global isNewEpisode
    global action_mins
    global action_maxs
    reset_model_state(startPoint)
    rospy.sleep(.25)
    #noise.reset()
    step = 1
    rate = rospy.Rate(10)
    episode_reward = 0
    print("isCollision = " + str(isCollision))
    while (not isCollision or isNewEpisode or (step > 5000)):
        if (step > 1):
            isNewEpisode = False
            reward = curReward
            agent.memory.push(oldState,action,reward,curState)
            episode_reward += reward
            print("reward = " + str(reward) + " | Episode reward = " + str(episode_reward))
        
        oldState = np.asarray(curState)
        action = agent.get_action(oldState)
        action[0] = np.clip(action[0],action_mins[0],action_maxs[0])
        action[1] = np.clip(action[1],action_mins[1],action_maxs[1])
        #print(str(action))
        action = noise.get_action(action,totalStep)
        #print(str(action))
        #print("Memory Buffer size = " + str(len(agent.memory)))
        actionTwist = Twist()
        actionTwist.linear.x = action[0]
        actionTwist.angular.z = action[1]
        cmd_vel_pub.publish(actionTwist)
        if (len(agent.memory) > batch_size):
            agent.update(batch_size)
        step+=1
        rate.sleep()
    episode_reward+=-1.0
    agent.memory.push(oldState,action,-1.0,curState)
    allEpisodeRewards.append(episode_reward)
    totalStep += step

collision_sub = rospy.Subscriber('/collision_detected',Bool,collision_callback)
model_state_sub = rospy.Subscriber('/gazebo/model_states',ModelStates,model_state_callback)
averaged_scan_sub = rospy.Subscriber('averaged_scan',AveragedScan,averaged_scan_callback)
current_reward_sub = rospy.Subscriber('current_reward',Float64,current_reward_callback)
agentFile = 'agent7.pkl'
rewardFile = 'agent7rewards.pkl'
if (path.exists('/home/ben/catkin_ws/src/rlracer/' + agentFile)):
    with open('/home/ben/catkin_ws/src/rlracer/' + agentFile,'rb') as f:
        agent = pickle.load(f)
    print(agentFile + ' file loaded')
else:
    agent = DDPGagent(360,256,1e-3,1e-3,.99,1e-2,500000)
    with open('/home/ben/catkin_ws/src/rlracer/' + agentFile,'w') as f:
        pickle.dump(agent,f)
    print(agentFile + ' file created')
if (path.exists('/home/ben/catkin_ws/src/rlracer/' + rewardFile)):
    with open('/home/ben/catkin_ws/src/rlracer/' + rewardFile,'rb') as f:
        allEpisodeRewards = pickle.load(f)
        print(rewardFile + " loaded")
else:
    allEpisodeRewards = []

while(curState == []):
    continue
noise = OUNoise(0.0,0.15,0.17,0.1,200000.0)
batch_size = 1024
rewards = []
avg_rewards = []
nEpisodes = 10000
plt.ion()
fig = plt.figure()
ax = fig.add_subplot(111)
startEp = len(allEpisodeRewards)
pltXs = range(1,startEp+1)
line1, = ax.plot(pltXs,allEpisodeRewards)
startPoint = 0
isNewEpisode = True
for i in range(0,nEpisodes):
    run_episode(agent,noise,startPoint)
    print("Episode #" + str(i+1) + " complete. Reward = " + str(allEpisodeRewards[startEp + i]) + " total time = " + str(totalStep) + " sigma = " + str(noise.sigma))
    pltXs.append(startEp + i + 1)
    line1.set_ydata(allEpisodeRewards)
    line1.set_xdata(pltXs)
    ax.relim()
    ax.autoscale_view()
    fig.canvas.draw()
    if (i > 0 and (i % 10) == 0):
        if (len(agent.memory) > batch_size):
            if (i % 50 == 0):
                for i in range(0,1000):
                    agent.update(batch_size)

                with open('/home/ben/catkin_ws/src/rlracer/' + agentFile,'w') as f:
                    pickle.dump(agent,f)

                with open('/home/ben/catkin_ws/src/rlracer/' + rewardFile,'w') as f:
                    pickle.dump(allEpisodeRewards,f);
                print('UPDATE DONE')
        startPoint = int(np.floor(np.random.rand()*len(xs)))
    isNewEpisode = True
    newEpisodeMsg = NewEpisode()
    newEpisodeMsg.is_new_episode = isNewEpisode
    newEpisodeMsg.start_point = startPoint
    new_episode_pub.publish(newEpisodeMsg)

with open('/home/ben/catkin_ws/src/rlracer/' + agentFile, 'w') as f:
    pickle.dump(agent,f)
print(agentFile + ' saved')

with open('/home/ben/catkin_ws/src/rlracer/' + rewardFile,'w') as f:
    pickle.dump(allEpisodeRewards,f)
print(rewardFile + ' saved')
