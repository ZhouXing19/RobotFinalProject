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

MOVING_TO_BASKET = "moving to basket"
MOVED_TO_BASKET = "moved to basket"
HOLDING_BASKET = "holding basket"
PULLING_BACK = "pulling basket"

class demoPickUp(object):

    def __init__(self, init_state=MOVING_TO_BASKET):

        self.initialized = False
        rospy.init_node('demo_pickup')
        
        self.rb0_scan_sub = rospy.Subscriber("/scan", LaserScan, self.scan_callback)
        self.rb0_cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)

        self.__goal_dist_in_front_basket = 0.24
        self.__prop = 0.10

        # The interface to the group of joints making up the turtlebot3
        #   openmanipulator arm
        self.move_group_arm = moveit_commander.MoveGroupCommander("arm")

        # The interface to the group of joints making up the turtlebot3
        #   openmanipulator gripper
        self.move_group_gripper = moveit_commander.MoveGroupCommander("gripper")

        # Initialize starting robot arm and gripper positions
        self.initialize_move_group()

        self.rb0_scan = []
        self.rb1_scan = []

        self.scan_max = None

        self.robot_status = MOVING_TO_BASKET

        if self.initialized == False:
            self.initialize_move_group()

        # Now everything is initialized
        self.initialized = True

    
    def initialize_move_group(self):
        arm_joint_goal = [0.0, 0.65, 0.15, -0.8]
        gripper_joint_goal = [0.015, 0.015]
        self.move_group_arm.go(arm_joint_goal, wait=True)
        self.move_group_gripper.go(gripper_joint_goal, wait=True)
        self.move_group_arm.stop()
        self.move_group_gripper.stop()
        rospy.sleep(0.5)

    def grab_basket(self):
        arm_joint_goal = [0.0, 0.05, -0.45, 0.4]
        gripper_joint_goal = [-0.01, -0.01]
        self.move_group_arm.go(arm_joint_goal, wait=True)
        self.move_group_arm.stop()
        self.move_group_gripper.go(gripper_joint_goal, wait=True)
        self.move_group_gripper.stop()
        self.robot_status = HOLDING_BASKET
        rospy.sleep(1.5)

    def set_vel(self, diff_ang=0.0, diff_dist=float('inf')):
        """ Set the velocities of the robot """

        ang_v, lin_v = None, None

        # Keep turning if the robot cannot see anything
        if diff_dist == float("inf"):
            print("=====I can't see it! Turning turning=====")
            ang_v = 0.15
            lin_v = 0.0
        
        # Stop if the robot is in front of the dumbbell/block
        elif diff_dist < self.__goal_dist_in_front_basket:
            print("=====Arrived!=====")
            ang_v, lin_v = 0.0, 0.0
        
        # Go forwards if robot is still away from dumbbell/block
        else:
            print("=====Rushing Ahead!=====")
            lin_v = self.__prop * diff_dist
            ang_v = 0.0
        
        return ang_v, lin_v
        

    def pub_vel(self, pub_channel, ang_v=0.0, lin_v=0.0):
        """ Publish a twist to the cmd_vel channel """

        # Set linear and angular velocities and publish
        new_twist = Twist()
        new_twist.linear.x = lin_v
        new_twist.angular.z = ang_v
        pub_channel.publish(new_twist)

    
    def stop_robot(self):
        self.pub_vel(self.rb0_cmd_vel_pub, 0.0, 0.0)
    
    def step_back(self):
        self.pub_vel(self.rb0_cmd_vel_pub, 0.0, -0.05)
        rospy.sleep(1)



    def scan_callback(self, data):
        if not self.initialized:
            return
        
        self.rb0_scan = data.ranges

        if not self.scan_max:
            self.scan_max = data.range_max
    

    
    def move_to_basket(self):
        # Now we are sure that the basket is in front of the robot

        if len(self.rb0_scan) == 0:
            print(" ---- no scan yet ---- ")
            return

        min_dist = min(self.rb0_scan[-10:] + self.rb0_scan[:10])
        print(f"---- min_dist ----: {min_dist}")
        
        if min_dist <= self.__goal_dist_in_front_basket:
            self.pub_vel(self.rb0_cmd_vel_pub, 0, 0)
            rospy.sleep(1)

            self.robot_status = MOVED_TO_BASKET
        else:

            ang_v, lin_v = self.set_vel(0, min_dist)
            self.pub_vel(self.rb0_cmd_vel_pub, ang_v, lin_v)
    
    def run(self):
        r = rospy.Rate(10)

        while not rospy.is_shutdown():

            if self.robot_status == MOVING_TO_BASKET:
                self.move_to_basket()
            elif self.robot_status == MOVED_TO_BASKET:
                self.grab_basket()
            elif self.robot_status == HOLDING_BASKET:
                self.step_back()
            elif self.robot_status == PULLING_BACK:
                self.stop_robot()
            
            r.sleep()



if __name__ == '__main__':
    try:
        node = demoPickUp()
        node.run()
    except rospy.ROSInterruptException:
        pass