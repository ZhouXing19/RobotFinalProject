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

MOVING_TO_BLOCKER = "moving to BLOCKER"
REACHED_BLOCKER = "moved to BLOCKER"
HOLDING_BLOCKER = "holding BLOCKER"
REACHED_DOOR = "reached door"


class demoLockDoors(object):

    def __init__(self, init_state=MOVING_TO_BLOCKER):

        self.initialized = False
        rospy.init_node('demo_pickup')


        self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback)
        self.scan_sub = rospy.Subscriber("/scan", LaserScan, self.scan_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)

        # Minimum distance in front of blocker and block
        self.__goal_dist_in_front__db = 0.22
        self.__prop = 0.15

        rospy.sleep(3)

        # The interface to the group of joints making up the turtlebot3
        #   openmanipulator arm
        self.move_group_arm = moveit_commander.MoveGroupCommander("arm")

        # The interface to the group of joints making up the turtlebot3
        #   openmanipulator gripper
        self.move_group_gripper = moveit_commander.MoveGroupCommander("gripper")

        # Initialize starting robot arm and gripper positions
        self.initialize_move_group()

        # Set up ROS / cv bridge
        self.bridge = cv_bridge.CvBridge()


        # Initialize array to hold and process images
        self.image = []

        self.__scan_data = []
        self.rb1_scan = []

        self.scan_max = None


        if self.initialized == False:
            self.initialize_move_group()
        
        self.robot_status = init_state

        self.curr_color = None

        # Now everything is initialized

        self.BLOCKER_status = {"BLOCKER_red": 0,
                "BLOCKER_white": 0,
                "BLOCKER_brown": 0}

        print("initalized!!!13333")
        self.initialized = True

    def scan_callback(self, data):
        if not self.initialized:
            return
        
        self.__scan_data = data.ranges

        if not self.scan_max:
            self.scan_max = data.range_max

    def image_callback(self, data):
        """ Process the image from the robot's RGB camera """

        # Do nothing if initialization is not done
        if not self.initialized:
            return

        # Take the ROS message with the image and turn it into a format cv2 can use
        self.image = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')

    def set_vel(self, diff_ang=0.0, diff_dist=float('inf')):
        """ Set the velocities of the robot """

        ang_v, lin_v = None, None

        # Keep turning if the robot cannot see anything
        if diff_dist == float("inf"):
            print("=====I can't see it! Turning turning=====")
            ang_v = 0.15
            lin_v = 0.0
        
        # Stop if the robot is in front of the blocker/block
        elif diff_dist < self.__goal_dist_in_front__db:
            print("=====Arrived!=====")
            ang_v, lin_v = 0.0, 0.0
        
        # Go forwards if robot is still away from blocker/block
        else:
            print("=====Rushing Ahead!=====")
            lin_v = self.__prop * diff_dist
            ang_v = 0.0
        
        return ang_v, lin_v
        

    def pub_vel(self, ang_v=0.0, lin_v=0.0):
        """ Publish a twist to the cmd_vel channel """

        # Set linear and angular velocities and publish
        self.twist.linear.x = lin_v
        self.twist.angular.z = ang_v
        self.cmd_vel_pub.publish(self.twist)

    def lift_blocker(self):
        """ Lift the blocker when robot reached the blockers """

        # Do nothing if the robot hasn't reached the blockers
        if self.robot_status != REACHED_BLOCKER:
            return 

        # Set arm and gripper joint goals and move them    
        arm_joint_goal = [0.0, 0.05, -0.45, 0.4]
        gripper_joint_goal = [0.004, 0.004]
        self.move_group_arm.go(arm_joint_goal, wait=True)
        self.move_group_gripper.go(gripper_joint_goal, wait=True)
        self.move_group_arm.stop()
        self.move_group_gripper.stop()

        # Step back so the robot won't hit the blocker while rotating
        print("----- stepping back!----")
        self.pub_vel(0, -0.5)
        rospy.sleep(0.8)
        self.pub_vel(0, 0)

        # After the robot grapped the blockers, it's time to identify the blocks
        self.robot_status = PICKED_UP_DB


    def drop_blocker(self):
        """ Drop the blocker when robot reached the blocks """

        # Do nothing is the robot hasn't reached the blocks
        if self.robot_status != REACHED_BLOCK:
            return

        # Set arm and gripper joint goals and move them
        arm_joint_goal = [0.0, 0.65, 0.15, -0.9]
        gripper_joint_goal = [0.015, 0.015]
        self.move_group_arm.go(arm_joint_goal, wait=True)
        rospy.sleep(0.5)
        self.move_group_gripper.go(gripper_joint_goal, wait=True)
        self.move_group_arm.stop()
        self.move_group_gripper.stop()
        rospy.sleep(1)

        # Step back so the robot won't hit the blocker while rotating
        print("----- stepping back!----")
        self.pub_vel(0, -0.3)
        rospy.sleep(1)
        self.pub_vel(0, 0)

        # After the robot dropped the blockers, it's time to go back to the blockers
        self.robot_status = GO_TO_DB

        # We also increase the number of action steps by 1
        self.action_step += 1

    def move_to_blocker(self, color: str):
        """ Move to a blocker based on color """

        # Do nothing if there are no images
        if len(self.image) == 0:
            print("-- Have not got the image --")
            return

        # Do nothing if there are no scan data
        if len(self.__scan_data) == 0:
            print("-- Have not got the scan --")
            return

        # Turn image into HSV style
        hsv = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)
        
        # Get lower and upper bounds of the specified color
        lb, ub = COLOR_BOUNDS[color]['lb'], COLOR_BOUNDS[color]['ub']
        
        # Mask and get moment of the color of the blocker
        mask = cv2.inRange(hsv, lb, ub)
        M = cv2.moments(mask)

        # Get the shape of the image to compute its center
        h, w, d = self.image.shape

        # If there are any pixels found for the desired color
        if M['m00'] > 0:

            # Determine the center of the colored pixels in the image
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])

            # blocker's distance to the center of the camera
            err = w/2 - cx

            print(f"abs(err) / w : {abs(err) / w}")

            # If the color center is at the front
            if abs(err) / w < 0.05:
                
                min_dist = min(self.__scan_data[-10:] + self.__scan_data[:10])

                print(f"min_dist: {min_dist}")

                # If the robot is close to the blocker
                if min_dist <= self.__goal_dist_in_front__db:

                    # Stop the robot
                    self.pub_vel(0,0)

                    # Sleep 1s to make sure the robot has stopped
                    rospy.sleep(1)

                    # Change status to reached blocker
                    self.robot_status = REACHED_DB
                    print(f"---reached blocker of color {color}----")

                    # Lift the blocker
                    self.lift_blocker()

                else:

                    # Rush STRAIGHT toward the blocker
                    ang_v, lin_v = self.set_vel(0, min_dist)
                    self.pub_vel(ang_v, lin_v)

            # If the color center is not right at the front yet
            else:
                
                # Define k_p for proportional control            
                k_p = 1.0 / 1000.0

                # Slowly turn the head, so that the color center 
                #   would be at the center of the camera
                self.pub_vel(k_p * err, 0)
                print(f"---turning to blocker of color {color}----")

        # If we cannot see any pixel of the desired color
        else:

            # Simply turn the head clockwise, without any linear speed
            ang_v, lin_v = self.set_vel()
            self.pub_vel(ang_v, lin_v)
