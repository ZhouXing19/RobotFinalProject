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

COLOR_BOUNDS = {
    'red_arc': {'lb': np.array([0, 220, 168]),
                'ub': np.array([20, 255, 230])},
    'blue_arc': {'lb': np.array([110, 235, 176]),
                'ub': np.array([130, 255, 255])},
    'green_arc': {'lb': np.array([50, 167, 176]), 
                'ub': np.array([70, 255, 255])},
    'yellow_arc': {'lb': np.array([30, 164, 110]), 
                'ub': np.array([60, 255, 255])}
}

FIND_ARC = "find archway"
APPROACH_ARC = "approach the archway and move through it"

class travel(object):
    
    def __init__(self):
        self.initialized = False
        rospy.init_node('travel')


        self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback)
        self.scan_sub = rospy.Subscriber("/scan", LaserScan, self.scan_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)

        # # Minimum distance in front of dumbbell
        # self.goal_dist_in_front_db = 0.23

        # self.__prop = 0.15

        rospy.sleep(3)

        # Set up ROS / cv bridge
        self.bridge = cv_bridge.CvBridge()


        # Initialize array to hold and process images
        self.image = []

        self.__scan_data = []
        self.rb1_scan = []

        self.scan_max = None


        
        self.robot_status = FIND_ARC
        
        self.curr_color = None
        # if self.initialized == False:
        #     self.find_arc(color)

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
    
    def set_vel(self, diff_ang=0.0, diff_dist=float('inf')):
            """ Set the velocities of the robot """

            ang_v, lin_v = None, None

            # Keep turning if the robot cannot see anything
            if diff_dist == float("inf"):
                print("=====I can't see it! Turning turning=====")
                ang_v = 0.15
                lin_v = 0.0
            
            # Stop if the robot is in front of the dumbbell/block
            elif diff_dist < self.goal_dist_in_front_db:
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
    
    # def step_back(self):
    #     self.pub_vel(0.0, -0.05)
    #     rospy.sleep(1)
            

    def find_arc(self, color: str):
        """ Turn towards an arc based off color """

        # Do nothing if there are no images
        if len(self.image) == 0:
            print("-- Have not got the image -- 137")
            return

        # Do nothing if there are no scan data
        if len(self.__scan_data) == 0:
            print("-- Have not got the scan -- 142")
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
            if abs(err) / w < 0.05:
                
                min_dist = min(self.__scan_data[-10:] + self.__scan_data[:10])

                print(f"min_dist: {min_dist}")

                # # If the robot is close to the dumbbell
                # if min_dist <= self.goal_dist_in_front_db:

                #     # Stop the robot
                #     self.pub_vel(0,0)

                #     # Sleep 1s to make sure the robot has stopped
                #     rospy.sleep(1)

                #     # Change status to reached dumbbell
                #     self.robot_status = REACHED_DUMBBELL
                #     print(f"---reached dumbbell of color {color}----")

                #     # Lift the dumbbell
                #     self.push_dumbbell()

                # else:

                #     # Rush STRAIGHT toward the dumbbell
                #     ang_v, lin_v = self.set_vel(0, min_dist)
                #     self.pub_vel(ang_v, lin_v)
                
                self.robot_status = APPROACH_ARC

            # If the color center is not right at the front yet
            else:
                
                # Define k_p for proportional control            
                k_p = 1.0 / 1000.0

                # Slowly turn the head, so that the color center 
                #   would be at the center of the camera
                self.pub_vel(k_p * err, 0)
                print(f"---turning to arc of color {color}----")

        # If we cannot see any pixel of the desired color
        else:

            # Simply turn the head clockwise, without any linear speed
            ang_v, lin_v = self.set_vel()
            self.pub_vel(ang_v, lin_v)
            
    def approach_arc(self, color:str):
        
        # Do nothing if there are no images
        if len(self.image) == 0:
            print("-- Have not got the image -- line 223")
            return

        # Do nothing if there are no scan data
        if len(self.__scan_data) == 0:
            print("-- Have not got the scan -- line 228")
            return
        
        # Turn image into HSV style
        hsv = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)
        
        # Get lower and upper bounds of the specified color
        lb, ub = COLOR_BOUNDS[color]['lb'], COLOR_BOUNDS[color]['ub']
        
        # Mask and get moment of the color of the dumbbell
        mask = cv2.inRange(hsv, lb, ub)
        M = cv2.moments(mask)
        
        
        while self.robot_status == APPROACH_ARC:
            
            mask = cv2.inRange(hsv, lb, ub)
            M = cv2.moments(mask)
            
            left_good = True
            right_good = True
            
            if 255 not in mask:
                print("no more pixels")
                self.pub_vel(0,0)
                self.robot_status == FIND_ARC
                break
            
            for i in range(-7,0):
                if self.__scan_data[i] != float("inf"):
                    left_good = False
                    break
            
            for i in range(0,7):
                if self.__scan_data[i] != float("inf"):
                    right_good = False
                    break
                
            if left_good and right_good:
                # while (M['m00'] > 0):
                # self.pub_vel(0,0)
                print("both good")
                self.pub_vel(0,.4)
                print("vel has been published")
                # self.pub_vel(0,0)
                
            elif left_good == False and right_good:
                print("left badd")
                self.pub_vel(0,0)
                self.pub_vel(0.1, 0.1,) 
                # self.pub_vel(0,0)
                
            elif left_good and right_good == False:
                print("right bad")
                self.pub_vel(0,0)
                self.pub_vel(-0.1, 0.1)
                # self.pub_vel(0,0)
            

    def run(self, color:str):
        r = rospy.Rate(5)

        while not rospy.is_shutdown():
            if self.robot_status ==  FIND_ARC:
                self.find_arc(color)
            elif self.robot_status == APPROACH_ARC:
                self.approach_arc(color)
            
            r.sleep()
            



if __name__ == '__main__':
    try:
        node = travel()
        node.run("red_arc")
    except rospy.ROSInterruptException:
        pass
            
            


        