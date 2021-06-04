#!/usr/bin/env python3

import numpy as np
import rospy, sys, os
from q_learning import QLearning

# Path of directory on where this file is located
path_prefix = os.path.dirname(__file__) + "/action_states/"

class Actions(object):

    def __init__(self):

        rospy.init_node("Actions")
        rospy.sleep(1)

        self.action_matrix = np.loadtxt(path_prefix + "action_matrix.txt")
        self.q_learning = QLearning()

        # Import qmatrix, action matrix and actions
        # import qmatrix
        self.qmatrix = []

        # create an empty action list of the optimal order of tasks
        self.action_list = []

        # Split the ordering of tasks into robot A and robot B tasks,
        # for the purposes of the demo
        # (As is probably known by now, our demo isn't collaborative)
        self.action_list_a = []
        self.action_list_b = []


        self.run()


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
        
        return

    def run(self):
        self.q_learning.read_q_matrix('a')
        self.read_matrix()
        print("done")
            
# run our node and keep rospy spinning
if __name__ == "__main__":
    node = Actions()
    rospy.spin()
