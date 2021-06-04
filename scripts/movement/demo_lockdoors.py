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

NOT_TURNED = "not turned"
MOVING_TO_BLOCKER = "moving to BLOCKER"
REACHED_BLOCKER = "moved to BLOCKER"
HOLDING_BLOCKER = "holding BLOCKER"
DROPPED_BLOCKER = "dropping BLOCKER"
TURNED_A_PI = "turned_a_pi"
FINISHED_FIRST_RUSHING = "finished_first_rushing"
TURNED_A_90 = "turned_a_90"
FINISHED_SECOND_RUSHING = "finished_second_rushing"

COLOR_BOUNDS = {
    'blocker_blue': {'lb': np.array([84, 125, 253]),
                'ub': np.array([87, 153, 255])},
    'green': {'lb': np.array([50, 50, 0]),
        'ub': np.array([100, 255, 255])},
    'blue': {'lb': np.array([100, 50, 0]), 
        'ub': np.array([160, 255, 255])},
}


class demoLockDoors(object):

    def __init__(self, init_state=NOT_TURNED):

        self.initialized = False
        rospy.init_node('demoLockDoors')

        self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback)
        self.scan_sub = rospy.Subscriber("/scan", LaserScan, self.scan_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)

        # Minimum distance in front of blocker and block
        self.__goal_dist_in_front__blocker = 0.19
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

        self.scan_max = None

        if self.initialized == False:
            self.initialize_move_group()
        
        self.robot_status = init_state

        # Now everything is initialized

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

    def set_vel(self, diff_ang=0.0, diff_dist=float('inf'), goal_dist=0.22):
        """ Set the velocities of the robot """

        ang_v, lin_v = None, None

        # Keep turning if the robot cannot see anything
        if diff_dist == float("inf"):
            print("=====I can't see it! Turning turning=====")
            ang_v = 0.15
            lin_v = 0.0
        
        # Stop if the robot is in front of the blocker/block
        elif diff_dist < goal_dist:
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
        new_twist = Twist()
        new_twist.linear.x = lin_v
        new_twist.angular.z = ang_v
        self.cmd_vel_pub.publish(new_twist)

    def stop_robot(self, period=1):
        self.pub_vel(0.0, 0.0)
        rospy.sleep(period)
    
    
    
    def step_back(self):
        self.pub_vel(0.0, -0.10)
        rospy.sleep(1)

    def first_rush(self, next_status=FINISHED_FIRST_RUSHING):
        self.pub_vel(0.0, 0.1)
        rospy.sleep(10)
        self.stop_robot(3)
        self.pub_vel(0.0, 0.1)
        rospy.sleep(13)
        self.stop_robot(3)
        print("end of first_rush")
        self.robot_status = next_status
    
    def second_rush(self, next_status=FINISHED_SECOND_RUSHING):
        self.pub_vel(0.0,0.1)
        rospy.sleep(15)
        self.stop_robot()
        print("end of second_rush")
        self.robot_status = next_status
    
    def turn_a_pi(self, next_status=TURNED_A_PI):
        self.pub_vel(0.157, 0.03)
        rospy.sleep(20)
        self.stop_robot()
        print("end of turn a pi")
        rospy.sleep(3)
        self.robot_status = next_status
    
    def turn_a_90(self, next_status=TURNED_A_90):
        self.pub_vel(0.157, 0.0)
        rospy.sleep(9.4)
        self.stop_robot()
        print("end of turn a 90")
        rospy.sleep(3)
        self.robot_status = next_status
    
    def initialize_move_group(self):
        """ Initialize the robot arm & gripper position so it can grab onto
        the dumbbell """

        # Set arm and gripper joint goals and move them
        arm_joint_goal = [0.0, 0.65, 0.15, -0.9]
        gripper_joint_goal = [0.015, 0.015]
        self.move_group_arm.go(arm_joint_goal, wait=True)
        self.move_group_gripper.go(gripper_joint_goal, wait=True)
        self.move_group_arm.stop()
        self.move_group_gripper.stop()

    def lift_blocker(self, next_status=HOLDING_BLOCKER):
        """ Lift the blocker when robot reached the blockers """

        # Do nothing if the robot hasn't reached the blockers
        if self.robot_status != REACHED_BLOCKER:
            return 

        # Set arm and gripper joint goals and move them  

        
        arm_joint_goal = [0.0, 0.05, -0.45, -0.1]
        gripper_joint_goal = [0.004, 0.004]

        print("===== lift1 ====")
        
        self.move_group_gripper.go(gripper_joint_goal, wait=True)
        print("===== lift2 ====")
        rospy.sleep(1.2)

        print("===== lift3 ====")
        self.move_group_arm.go(arm_joint_goal, wait=True)

        print("===== lift4 ====")

        self.move_group_gripper.stop()
        print("===== lift5 ====")
        self.move_group_arm.stop()

        print("===== lift6 ====")
        rospy.sleep(1)

        print("===== lift7 ====")
        

        # After the robot grapped the blockers, it's time to identify the blocks
        self.robot_status = next_status
    
    def rotate_arm(self):
        arm_joint_goal = [2.0, 0.05, -0.45, -0.1]
        self.move_group_arm.go(arm_joint_goal, wait=True)
        gripper_joint_goal = [0.004, 0.004]
        self.move_group_gripper.go(gripper_joint_goal, wait=True)
        self.move_group_arm.stop()
        self.move_group_gripper.stop()

    def drop_blocker(self, next_status=DROPPED_BLOCKER):
        """ Drop the blocker when robot reached the blocks """

        # Set arm and gripper joint goals and move them
        arm_joint_goal = [0.0, 0.65, 0.15, -0.9]
        gripper_joint_goal = [0.015, 0.015]
        self.move_group_arm.go(arm_joint_goal, wait=True)
        rospy.sleep(1)
        self.move_group_gripper.go(gripper_joint_goal, wait=True)
        self.move_group_arm.stop()
        self.move_group_gripper.stop()
        rospy.sleep(1)


        # After the robot dropped the blockers, it's time to go back to the blockers
        self.robot_status = next_status

    def move_to_object(self, color, goal_dist=0.215, next_status=REACHED_BLOCKER):
        """ Move to a dumbbell based on color """

        # Do nothing if there are no images
        if len(self.image) == 0:
            print("-- Have not got the image --")
            return

        # Do nothing if there are no scan data
        if len(self.__scan_data) == 0:
            print("-- Have not got the scan --")
            return
        
        if self.initialized == False:
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
            
            print("--- Find my color ---")
            # Determine the center of the colored pixels in the image
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])

            # Dumbbell's distance to the center of the camera
            err = w/2 - cx

            print(f"abs(err) / w : {abs(err) / w}")

            # If the color center is at the front
            if abs(err) / w <= 0.05:
                
                min_dist = min(self.__scan_data[-10:] + self.__scan_data[:10])

                print(f"min_dist: {min_dist}")

                # If the robot is close to the dumbbell
                if min_dist <= goal_dist:

                    # Stop the robot
                    self.pub_vel(0,0)

                    # Sleep 1s to make sure the robot has stopped
                    rospy.sleep(2)

                    print(f"---reached dumbbell of color {color}----")
                    
                    # Change status to reached dumbbell
                    self.robot_status = next_status
                    

                else:

                    # Rush STRAIGHT toward the dumbbell
                    ang_v, lin_v = self.set_vel(0, min_dist, goal_dist)
                    self.pub_vel(ang_v, lin_v)

            # If the color center is not right at the front yet
            else:
                
                # Define k_p for proportional control            
                k_p = 1.0 / 1000.0

                # Slowly turn the head, so that the color center 
                #   would be at the center of the camera
                self.pub_vel(k_p * err, 0)
                print(f"---turning to dumbbell of color {color}----")

        # If we cannot see any pixel of the desired color
        else:

            # Simply turn the head clockwise, without any linear speed
            ang_v, lin_v = self.set_vel()
            self.pub_vel(ang_v, lin_v)
    
    def run(self):
        r = rospy.Rate(10)

        while not rospy.is_shutdown():
            if self.robot_status == NOT_TURNED:
                self.pub_vel(ang_v = 0.0, lin_v = 0.5)
                rospy.sleep(2.5)
                self.stop_robot()
                self.pub_vel(ang_v = -0.314, lin_v = 0.0)
                rospy.sleep(5)
                self.stop_robot()
                self.robot_status = MOVING_TO_BLOCKER
            elif self.robot_status == MOVING_TO_BLOCKER:

                self.move_to_object("blocker_blue", 
                                    goal_dist=self.__goal_dist_in_front__blocker, 
                                    next_status=REACHED_BLOCKER)
            elif self.robot_status == REACHED_BLOCKER:
                self.lift_blocker(next_status=HOLDING_BLOCKER)
                print("holding now")
            elif self.robot_status == HOLDING_BLOCKER:
                self.turn_a_pi(next_status=TURNED_A_PI)
            elif self.robot_status == TURNED_A_PI:
                self.first_rush(next_status=FINISHED_FIRST_RUSHING)
                print("finished first rush")
            elif self.robot_status == FINISHED_FIRST_RUSHING:
                self.turn_a_90(next_status=TURNED_A_90)
                print("turned a 90")
            elif self.robot_status == TURNED_A_90:
                self.second_rush(next_status=FINISHED_SECOND_RUSHING)
                print("finished second rush")
            elif self.robot_status == FINISHED_SECOND_RUSHING:
                print("==== hi ======")
                self.turn_a_pi(next_status=FINISHED_SECOND_RUSHING)
                self.drop_blocker(next_status=DROPPED_BLOCKER)
            elif self.robot_status == DROPPED_BLOCKER:
                print("====drop===")
                self.step_back()
            r.sleep()

if __name__ == '__main__':
    print("running")
    try:
        node = demoLockDoors()
        node.run()
    except rospy.ROSInterruptException:
        pass