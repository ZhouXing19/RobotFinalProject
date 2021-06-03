#!/usr/bin/env python3

import rospy


from gazebo_msgs.msg import ModelState, ModelStates
from escapebots.msg import QLearningReward
from escapebots.msg import RobotTasksDoors

from std_msgs.msg import Header

from random import shuffle


class ResetWorld(object):

    def __init__(self):

        # initialize this node
        rospy.init_node('virtual_reset_world_final_proj')

        # reward amounts
        self.positive_reward = 100
        self.negative_reward = 0

        # goal completion status
        self.goal_robot_task_door = {
            0: 1,
            1: 1, 
            2: 1,
            3: 1,
            4: 1,
            5: 1,
            6: 1
        }

        # current status of the robot's tasks and doors
        self.current_robot_task_door = {
            0: 0,
            1: 0, 
            2: 0,
            3: 0,
            4: 0,
            5: 0,
            6: 0
        }

        # keep track of the iteration number
        self.iteration_num = 0

        # TODO: ROS subscribe to the topic publishing actions for the robot to take
        rospy.Subscriber("/RobotFinalProject/robot_action", RobotTasksDoors, self.send_reward)

        # ROS publishers
        self.reward_pub = rospy.Publisher("/RobotFinalProject/reward", QLearningReward, queue_size=10)

        self.run()

    def send_reward(self, data):
        print(data)
        # update whether a task is complete / door is closed
        self.current_robot_task_door[data.task_or_door] = data.is_complete

        # default reward
        reward_amount = self.positive_reward
        reset_world = True

        # check if dbs are in correct position & if the world should be reset
        for robot_task_door in self.current_robot_task_door.keys():
            if self.current_robot_task_door[robot_task_door] != self.goal_robot_task_door[robot_task_door]:
                reward_amount = self.negative_reward


            if self.current_robot_task_door[robot_task_door] == 0:
                reset_world = False


        # prepare reward msg
        reward_msg = QLearningReward()
        reward_msg.header = Header(stamp=rospy.Time.now())
        reward_msg.reward = reward_amount
        reward_msg.iteration_num = self.iteration_num
        self.reward_pub.publish(reward_msg)
        print("Published reward: ", reward_amount)

        # increment iteration if world needs to be reset
        # reset task completion status if world needs to be rest
        if reset_world:
            print("resetting the world")
            self.iteration_num += 1

            for robot_task_door in self.current_robot_task_door.keys():
                self.current_robot_task_door[robot_task_door] = 0

    def run(self):
        rospy.spin()

if __name__=="__main__":

    node = ResetWorld()
