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

path_prefix = os.path.dirname(__file__) + "/action_states/"

action_dict = {0: "close red door",
1: "close yellow door",
2: "close blue door",
3: "close green door",
4: "hang bears",
5: "dumbbells",
6: "dancing"}


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


class Actions_rb2(object):

    def __init__(self, init_state=NOT_TURNED):

        self.initialized = False
        rospy.init_node('Actions_rb2')

        self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback)
        self.scan_sub = rospy.Subscriber("/scan", LaserScan, self.scan_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)

        # Minimum distance in front of blocker and block
        self.__goal_dist_in_front__blocker = 0.20
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

        self.robot_status = init_state

        # Now everything is initialized

        self.action_matrix = np.loadtxt(path_prefix + "action_matrix.txt")

        # Import qmatrix, action matrix and actions
        # import qmatrix
        self.qmatrix = self.read_q_matrix('a')

        # create an empty action list of the optimal order of tasks
        self.action_list = []

        # Split the ordering of tasks into robot A and robot B tasks,
        # for the purposes of the demo
        # (As is probably known by now, our demo isn't collaborative)
        self.action_list_a = []
        self.action_list_b = []

        # Get the action sequence for robot a and robot b
        self.read_matrix()

        self.initialized = True

    def read_matrix(self):
        # initialize state to 0
        final_state = False
        curr_state = 0

        # while the reward of 100 hasn't been found:
        # find the next action
        while not final_state:
            # create an array of possible actions at the state and chose the action with the highest reward
            poss_actions = self.qmatrix[curr_state]
            next_action = np.argmax(poss_actions)

            # add the action to our action list
            self.action_list.append(next_action)

            # if the reward for this action is 100, we are done
            if int(self.qmatrix[curr_state][next_action]) == 100:
                final_state = True

            # calculate the next state of the robot after perfoming the previous action
            curr_state = np.where(self.action_matrix[curr_state] == next_action)[0][0]

        print("action list:")
        print(self.action_list)

        for action in self.action_list:
            if action >= 4:
                self.action_list_a.append(action)
            else:
                self.action_list_b.append(action)
        
        print("a list:")
        for i in self.action_list_a:
            print(f"{action_dict[i]}", end=', ')

        print()
        
        print("b list:")
        for i in self.action_list_b:
            print(f"{action_dict[i]}", end=', ')
        
        return


    def read_q_matrix(self, agent):
        # Save the Q-matrix as a 2D float array
        q_matrix_arr = []

        # Read in the .txt file
        with open(os.path.dirname(__file__) + '/q_matrix_' + agent + '.txt','r') as f:
            for line in f.readlines():
                # The code below is messy...
                # ...but it converts all those string arrays in the .txt to float arrays
                # (We could've just used JSON. We were devoted to .txt though)
                q_matrix_arr.append([float(x) for x in line[0:-1].split(', ')])

        return q_matrix_arr


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
    
    def step_back(self, period=6):
        self.pub_vel(0.0, -0.50)
        rospy.sleep(period)
        self.stop_robot()
        print("=====stepped_back=====")

    def first_rush(self, next_status=FINISHED_FIRST_RUSHING):
        self.pub_vel(0.0, 0.1)
        rospy.sleep(12)
        self.stop_robot(3)
        self.pub_vel(0.0, 0.1)
        rospy.sleep(11)
        self.stop_robot(3)
        print("end of first_rush")
        self.robot_status = next_status
    
    def second_rush(self, next_status=FINISHED_SECOND_RUSHING):
        self.pub_vel(0.0,0.1)
        rospy.sleep(16)
        self.stop_robot()
        print("end of second_rush")
        self.robot_status = next_status
    
    def turn_a_pi(self, next_status=TURNED_A_PI, lin_speed=0.03):
        self.pub_vel(0.157, lin_speed)
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
    
    def turn_a_degree(self, degree):
        euler = (degree / 180) * 3.14
        default_ang_speed = 0.157 
        self.pub_vel(default_ang_speed, 0.0)
        rospy.sleep(euler/default_ang_speed)
        self.stop_robot()
        rospy.sleep(3)

    def initialize_move_group(self):
        """ Initialize the robot arm & gripper position so it can grab onto
        the dumbbell """

        # Set arm and gripper joint goals and move them
        arm_joint_goal = [0.0, 0.725, 0.045, -0.8]
        gripper_joint_goal = [0.016, 0.016]
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
        self.move_group_gripper.go(gripper_joint_goal, wait=True)
        rospy.sleep(1.2)
        self.move_group_arm.go(arm_joint_goal, wait=True)
        self.move_group_gripper.stop()
        self.move_group_arm.stop()
        rospy.sleep(1)
        

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
    
    def lock_one_door(self):
        if not self.initialized:
            return
        if self.robot_status == NOT_TURNED:
            self.step_back(7)
            self.turn_a_90()
            self.initialize_move_group()
            # # origin -> arch
            self.pub_vel(0, 0.5)
            rospy.sleep(8)
            self.stop_robot()
            print("====== arrived at arch =====")
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
            self.turn_a_pi(next_status=FINISHED_SECOND_RUSHING, lin_speed=0.0)
            self.drop_blocker(next_status=DROPPED_BLOCKER)
        elif self.robot_status == DROPPED_BLOCKER:
            print("====drop===")
            # arch -> origin
            self.step_back()
            self.turn_a_degree(120)
            self.robot_status = NOT_TURNED
    
    def run(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.lock_one_door()
            r.sleep()

if __name__ == '__main__':
    print("running")
    try:
        node = Actions_rb2()
        node.run()
    except rospy.ROSInterruptException:
        pass