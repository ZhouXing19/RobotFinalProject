#!/usr/bin/env python3

import rospy, rospkg, cv2, cv_bridge
import os
import numpy as np
import math

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

import moveit_commander

'''
close green door, close blue door, close red door, close yellow door, done
'''

