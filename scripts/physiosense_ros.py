#!/usr/bin/env python2

import rospy
import sys
import copy
import math
import numpy as np
import actionlib
import geometry_msgs.msg
import socket
import struct
from math import pi
from std_msgs.msg import String, Header
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from std_msgs.msg import(
    Int32,
)

def main():
   
    rospy.init_node('physiosense')

    #rospy.Subscriber("/task_id_robot", Int32, updateTaskId)
    #pubRobotState = rospy.Publisher('/robot_state', Int32, queue_size = 2, latch=False)

    print "Launching physiosense..."

    while rospy.is_shutdown() == False:
        rospy.sleep(0.5)

if __name__ == '__main__':
    main()
