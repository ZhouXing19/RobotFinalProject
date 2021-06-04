#!/usr/bin/env python3

import numpy as np
import rospy, sys, os

# Path of directory on where this file is located
path_prefix = os.path.dirname(__file__) + "/action_states/"
print(f"path_prefix:{path_prefix}")

print("running action.py")

action_dict = {0: "close red door",
1: "close yellow door",
2: "close blue door",
3: "close green door",
4: "hang bears",
5: "dumbbells",
6: "dancing"}

class Actions(object):

    def __init__(self):

        
        self.initialized = False
        rospy.init_node("Actions")
        

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

    def run(self):
        if not self.initialized:
            return
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.read_matrix()
            print("done")
            r.sleep()
        
                
# run our node and keep rospy spinning
if __name__ == '__main__':
    try:
        node = Actions()
        node.run()
        
    except rospy.ROSInterruptException:

        pass
