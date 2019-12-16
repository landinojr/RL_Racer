#! /usr/bin/env python2
import rospy
import math
from rlracer.srv import reward_request,reward_requestRequest,reward_requestResponse
xs = [-3.000, -2.674, -2.453, -2.172, -2.018, -1.772, -1.307, -.933, -.499, .003, .465, .890, 1.354, 1.756, 2.506, 2.946, 3.293, 3.030, 2.630, 1.785, .889, -.110, -.773, -1.006, -1.009, -1.159, -1.896, -2.661, -2.891, -3.092, -3.191]

ys = [1.500, 1.500, 1.500, 1.550, 1.601, 1.735, 1.739, 1.447, 1.138, .849, .719, .729, .743, .959,1.027, .834, .348, -.485, -.599, -.524, -.486, -.485, -.619, -1.008, -1.663, -2.077, -2.040, -1.626, -.912, -.209, .644]

rospy.init_node('RewardServer')
curDistance = 0
timeCost = -.001
progressReward = .01
while (rospy.Time.now().to_sec() == 0):
    continue
checkpointStart = rospy.Time.now().to_sec()

def getDistanceSum(checkpt,dist2nextChkPt):
    d = 0
    for i in range(0,checkpt):
        if (i < checkpt-1):
            d += math.sqrt((xs[i] - xs[i+1])**2 + (ys[i] - ys[i+1])**2)
        else:
            d -= dist2nextChkPt
    return d
#print(str(checkpointStart))
def computeReward(msg):
    global curDistance
    global checkpointStart
    global timeCost
    global xs
    global ys
    global progressReward
    if (msg.new_episode):
        checkpointStart = rospy.Time.now().to_sec()
    if (msg.checkpoint == len(xs)):
        cpx = xs[0]
        cpy = ys[0]
    else:
        cpx = xs[msg.checkpoint]
        cpy = ys[msg.checkpoint]
    #print("(" + str(cpx) + "," + str(cpy) + ")    (" + str(msg.x) + "," + str(msg.y) + ")")
    d = math.sqrt((cpx - msg.x)**2 + (cpy - msg.y)**2)
    if (d < curDistance):
        curDistance = d
        curTime = rospy.Time.now().to_sec()
        timeInCheckpoint = curTime - checkpointStart
        #r = msg.checkpoint*1.0 - timeInCheckpoint*timeCost
        #r = getDistanceSum(msg.checkpoint,d) - timeInCheckpoint*timeCost
        if (d < msg.min_distance):
            increment_checkpoint = True
            r = 1.0
            checkpointStart = rospy.Time.now().to_sec()
            print("RewardServer: msg.checkpoint = " + str(msg.checkpoint))
            if (msg.checkpoint == (len(xs)-1)):
                cpx = xs[0]
                cpy = ys[0]
            else:
                cpx = xs[msg.checkpoint+1]
                cpy = ys[msg.checkpoint+1]
            curDistance = math.sqrt((cpx - msg.x)**2 + (cpy - msg.y)**2)
        else:
            increment_checkpoint = False
            r = progressReward + timeCost
        response = reward_requestResponse()
        response.reward = r
        response.nextCheckpoint = increment_checkpoint
        response.timeInCheckpoint = timeInCheckpoint
        response.distance = d
        return response
    else:
        curDistance = d
        curTime = rospy.Time.now().to_sec()
        timeInCheckpoint = curTime - checkpointStart
        #print(str(checkpointStart))
        #print(str(curTime))
        #print(str(timeInCheckpoint))
        #r = -1.0 - timeInCheckpoint*timeCost
        #r = getDistanceSum(msg.checkpoint,d) - timeInCheckpoint*timeCost
        response = reward_requestResponse()
        response.reward = -progressReward + timeCost
        response.nextCheckpoint = False
        response.timeInCheckpoint = timeInCheckpoint
        response.distance = d
        return response

service = rospy.Service('reward_server',reward_request,computeReward)
rospy.spin()
