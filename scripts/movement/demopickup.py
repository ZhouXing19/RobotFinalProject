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
red bear: lower_red = np.array([0, 50, 150])
upper_red = np.array([20, 255, 255])

brown bear: lower_red = np.array([2, 219, 20])
upper_red = np.array([22, 259, 120])

while bear: lower_red = np.array([0, 0, 255])
upper_red = np.array([150, 150, 255])
'''


# Define colors for the dumbbells and block number
COLOR_BOUNDS = {
    'red': {'lb': np.array([0, 50, 0]),
                'ub': np.array([50, 255, 255])},
    'green': {'lb': np.array([50, 50, 0]),
            'ub': np.array([100, 255, 255])},
    'blue': {'lb': np.array([100, 50, 0]), 
            'ub': np.array([160, 255, 255])},
    'black': {'lb': np.array([0, 0, 0]), 
            'ub': np.array([180, 255, 50])},
    'bear_red': {'lb': np.array([0, 50, 150]),
                'ub': np.array([20, 255, 255])},
    'bear_brown': {'lb': np.array([2, 219, 20]),
                'ub': np.array([22, 259, 120])},
    'bear_white': {'lb': np.array([0, 0, 255]),
                'ub': np.array([150, 150, 255])}
}

MOVING_TO_BEAR = "moving to BEAR"
MOVED_TO_BEAR = "moved to BEAR"
HOLDING_BEAR = "holding BEAR"
PULLING_BACK = "pulling BEAR"
REACHED_SUPPORTER_RED = "REACHED_SUPPORTER_RED"
REACHED_SUPPORTER_BROWN = "REACHED_SUPPORTER_BROWN"
REACHED_SUPPORTER_WHITE = "REACHED_SUPPORTER_WHITE"

# Define robot statuses to keep track of its actions
MEASURE_ANGLE = "measure_angle"
GO_TO_DB = "go_to_dumbbell"
REACHED_DB = "reached_db"
PICKED_UP_DB = "picked_up_dumbbell"
MOVING_TO_BLOCK = "moving_to_block"
REACHED_BLOCK = "reached_block"

class demoPickUp(object):

    def __init__(self, init_state=MOVING_TO_BEAR):

        self.initialized = False
        rospy.init_node('demo_pickup')


        self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback)
        self.scan_sub = rospy.Subscriber("/scan", LaserScan, self.scan_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)

        # Minimum distance in front of dumbbell and block
        self.__goal_dist_in_front__db = 0.23

        self.__goal_dist_in_front_BEAR = 0.217
        self.__prop = 0.17

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

        self.bear_status = {"bear_red": 0,
                "bear_white": 0,
                "bear_brown": 0}

        print("initalized!!!13333")
        self.initialized = True

    
    def initialize_move_group(self):
        """ Initialize the robot arm & gripper position so it can grab onto
        the dumbbell """

        # Set arm and gripper joint goals and move them
        arm_joint_goal = [0.0, 0.725, 0.045, -0.8]
        gripper_joint_goal = [0.015, 0.015]
        self.move_group_arm.go(arm_joint_goal, wait=True)
        self.move_group_gripper.go(gripper_joint_goal, wait=True)
        self.move_group_arm.stop()
        self.move_group_gripper.stop()
        rospy.sleep(0.5)
    
    def lift_dumbbell(self, next_status):
        """ Lift the dumbbell when robot reached the dumbbells """

        print("----lifting-----")

        # Set arm and gripper joint goals and move them    
        arm_joint_goal = [0.0, 0.16, -0.500, -0.10]
        gripper_joint_goal = [0.004, 0.004]
        self.move_group_arm.go(arm_joint_goal, wait=True)
        self.move_group_gripper.go(gripper_joint_goal, wait=True)
        self.move_group_arm.stop()
        self.move_group_gripper.stop()


        # After the robot grapped the dumbbells, it's time to identify the blocks
        self.robot_status = next_status


    def grab_bear(self):
        arm_joint_goal = [0.0, 0.05, -0.45, 0.4]
        gripper_joint_goal = [0.004, 0.004]
        self.move_group_arm.go(arm_joint_goal, wait=True)
        self.move_group_gripper.go(gripper_joint_goal, wait=True)
        self.move_group_arm.stop()
        self.move_group_gripper.stop()
        rospy.sleep(2)
        self.robot_status = HOLDING_BEAR

    def drop_bear(self):
        arm_joint_goal = [0.0, 0.25, -0.15, 0.2]
        gripper_joint_goal = [0.01, 0.01]
        self.move_group_arm.go(arm_joint_goal, wait=True)
        self.move_group_gripper.go(gripper_joint_goal, wait=True)
        self.move_group_arm.stop()
        self.move_group_gripper.stop()
        rospy.sleep(2)
        print("---- Released gripper and lowered arms ----")
        
    def set_vel(self, diff_ang=0.0, diff_dist=float('inf')):
        """ Set the velocities of the robot """

        ang_v, lin_v = None, None

        # Keep turning if the robot cannot see anything
        if diff_dist == float("inf"):
            print("=====I can't see it! Turning turning=====")
            ang_v = 0.50
            lin_v = 0.10
        
        # Stop if the robot is in front of the dumbbell/block
        elif diff_dist < self.__goal_dist_in_front__db:
            print("=====Arrived!=====")
            ang_v, lin_v = 0.0, 0.0
        
        # Go forwards if robot is still away from dumbbell/block
        else:
            print("=====Rushing Ahead!=====")
            lin_v = self.__prop * diff_dist
            ang_v = 0.0
        
        return ang_v, lin_v
        

    def pub_vel(self, ang_v=0.0, lin_v=0.0):
        """ Publish a twist to the cmd_vel channel """

        # Set linear and angular velocities and publish
        new_twist = Twist()
        new_twist.linear.x = lin_v
        new_twist.angular.z = ang_v
        self.cmd_vel_pub.publish(new_twist)


    def stop_robot(self):
        self.pub_vel(0.0, 0.0)
    
    def step_back(self):
        self.pub_vel(0.0, -0.25)
        rospy.sleep(0.4)



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

    
    
    def move_to_object(self, color, goal_dist=0.20, next_status=MOVED_TO_BEAR):
        """ Move to a dumbbell based on color """

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
        
        # Mask and get moment of the color of the dumbbell
        mask = cv2.inRange(hsv, lb, ub)
        M = cv2.moments(mask)

        # Get the shape of the image to compute its center
        h, w, d = self.image.shape

        # If there are any pixels found for the desired color
        if M['m00'] > 0:

            # Determine the center of the colored pixels in the image
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])

            # Dumbbell's distance to the center of the camera
            err = w/2 - cx

            print(f"abs(err) / w : {abs(err) / w}")

            # If the color center is at the front
            if abs(err) / w <= 0.052:
                
                min_dist = min(self.__scan_data[-10:] + self.__scan_data[:10])

                print(f"min_dist: {min_dist}")

                # If the robot is close to the dumbbell
                if min_dist <= goal_dist:

                    # Stop the robot
                    self.pub_vel(0,0)

                    # Sleep 1s to make sure the robot has stopped
                    rospy.sleep(1)

                    print(f"---reached dumbbell of color {color}----")
                    
                    # Change status to reached dumbbell
                    self.robot_status = next_status
                    

                else:

                    # Rush STRAIGHT toward the dumbbell
                    ang_v, lin_v = self.set_vel(0, min_dist)
                    self.pub_vel(ang_v, lin_v)

            # If the color center is not right at the front yet
            else:
                
                # Define k_p for proportional control            
                k_p = 1.0 / 2000.0

                # Slowly turn the head, so that the color center 
                #   would be at the center of the camera
                self.pub_vel(k_p * err, 0)
                print(f"---turning to dumbbell of color {color}----")

        # If we cannot see any pixel of the desired color
        else:

            # Simply turn the head clockwise, without any linear speed
            ang_v, lin_v = self.set_vel()
            self.pub_vel(ang_v, lin_v)
    
    def lift_to_supporter(self, next_status=MOVING_TO_BEAR):
        print("==========lift_to_supporter===========")
        self.drop_bear()
        self.step_back()
        self.bear_status[self.curr_color] = 1
        self.robot_status = next_status



    
    def run(self):
        r = rospy.Rate(5)

        while not rospy.is_shutdown():

            if self.bear_status['bear_red'] == 0:
                self.curr_color = 'bear_red'
            elif self.bear_status['bear_white'] == 0:
                self.curr_color = 'bear_white'
            else:
                self.curr_color = 'bear_brown'

            print(f"**** current color: {self.curr_color} ****")

            if self.robot_status == MOVING_TO_BEAR:
                self.move_to_object(color=self.curr_color, goal_dist=self.__goal_dist_in_front_BEAR, next_status=MOVED_TO_BEAR)
            elif self.robot_status == MOVED_TO_BEAR:
                self.lift_dumbbell(next_status=HOLDING_BEAR)
            elif self.robot_status == HOLDING_BEAR:
                self.move_to_object(color=self.curr_color, goal_dist=0.38, next_status=REACHED_SUPPORTER_RED)
            elif self.robot_status == REACHED_SUPPORTER_RED:
                self.lift_to_supporter()
                self.initialize_move_group()
                


                
            r.sleep()



if __name__ == '__main__':
    try:
        node = demoPickUp()
        node.run()
    except rospy.ROSInterruptException:
        pass