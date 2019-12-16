#! /usr/bin/env python2
import rospy
from gazebo_msgs.msg import ModelStates
from rlracer.srv import reward_request,reward_requestRequest,reward_requestResponse
from rlracer.msg import NewEpisode
import math
from std_msgs.msg import Float64,Bool,Int16

rospy.init_node('GazeboListener')
rospy.wait_for_service('reward_server')
reward_server = rospy.ServiceProxy('reward_server',reward_request)
reward_pub = rospy.Publisher('current_reward',Float64,queue_size=10)
checkpoint_pub = rospy.Publisher('current_checkpoint',Int16,queue_size=10)
xs = [-3.000, -2.674, -2.453, -2.172, -2.018, -1.772, -1.307, -.933, -.499, .003, .465, .890, 1.354, 1.756, 2.506, 2.946, 3.293, 3.030, 2.630, 1.785, .889, -.110, -.773, -1.006, -1.009, -1.159, -1.896, -2.661, -2.891, -3.092, -3.191]

ys = [1.500, 1.500, 1.500, 1.550, 1.601, 1.735, 1.739, 1.447, 1.138, .849, .719, .729, .743, .959,1.027, .834, .348, -.485, -.599, -.524, -.486, -.485, -.619, -1.008, -1.663, -2.077, -2.040, -1.626, -.912, -.209, .644]

curCheckpoint=1
curDistance=0
curX = xs[0]
curY = ys[0]
isNewEpisode = False
def model_state_callback(msg):
    global curDistance
    global curX
    global curY
    for ind,name in enumerate(msg.name):
        if (name == 'turtlebot3_burger'):
            curX = msg.pose[ind].position.x
            curY = msg.pose[ind].position.y

def new_episode(msg):
    global isNewEpisode
    global curCheckpoint
    global xs
    isNewEpisode = msg.is_new_episode
    startPoint = msg.start_point
    if (startPoint >= (len(xs)-1)):
        curCheckpoint = 0
    else:
        curCheckpoint = startPoint+1

sub = rospy.Subscriber('/gazebo/model_states',ModelStates,model_state_callback)
new_episode_sub = rospy.Subscriber('/is_new_episode',NewEpisode,new_episode)
rate = rospy.Rate(10)
while not rospy.is_shutdown():
    request = reward_requestRequest()
    request.x = curX
    request.y = curY
    request.checkpoint = curCheckpoint
    request.min_distance = .2
    request.new_episode = isNewEpisode
    if (isNewEpisode):
        isNewEpisode = False
    response = reward_server(request)
    if (response.nextCheckpoint):
        if (curCheckpoint == (len(xs)-1)):
            curCheckpoint = 0
        else:
            curCheckpoint+=1
    reward_pub.publish(response.reward)
    checkpoint_pub.publish(curCheckpoint)
    #print("reward = " + str(response.reward) + " | checkpoint = " + str(curCheckpoint) + " | T(c) = " + str(response.timeInCheckpoint) + " | distance = " + str(response.distance))
   #print("Current checkpoint = " + str(curCheckpoint))
    rate.sleep()

