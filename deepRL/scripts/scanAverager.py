#!/usr/bin/env python2
import rospy
from rlracer.msg import AveragedScan
from sensor_msgs.msg import LaserScan
import numpy as np

rospy.init_node('scanAverager')

scanBuffer = []
startup = True
bufferLength = 5
averaged_scan_pub = rospy.Publisher('averaged_scan', AveragedScan, queue_size=10)
def scan_callback(msg):
    global startup
    global scanBuffer
    global bufferLength
    # Initialize scanBuffer using length of LaserScan.ranges
    if (startup):
        scanBuffer = np.zeros((bufferLength,len(msg.ranges)))
        startup = False
    scanBuffer[0:(scanBuffer.shape[0]-2)][:] = scanBuffer[1:(scanBuffer.shape[0]-1)][:]
    zeroInds = np.where(msg.ranges == 0)[0]
    scan = np.asarray(msg.ranges)
    scan[zeroInds.tolist()] = float('inf')
    infInds = np.where(scan == float('inf'))[0]
    scan[infInds.tolist()] = 10
    scanBuffer[bufferLength-1][:] = scan
    averaged_scan_pub.publish(np.nanmean(scanBuffer,0))

scan_sub = rospy.Subscriber('/scan', LaserScan, scan_callback)
rospy.spin()
