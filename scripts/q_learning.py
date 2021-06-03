#!/usr/bin/env python3

import rospy
import numpy as np
import os
# TODO: escapebots.msg cannot be imported...still figuring this out
from escapebots.msg import QMatrix, QMatrixRow

# Path of directory on where this file is located
path_prefix = os.path.dirname(__file__) + "/action_states/"

class QLearning(object):
    def __init__(self):
        # Initialize this node
        rospy.init_node("q_learning")

        # Fetch pre-built action matrix. This is a 2d numpy array where row indexes
        # correspond to the starting state and column indexes are the next states.
        #
        # A value of -1 indicates that it is not possible to get to the next state
        # from the starting state. Values 0-6 correspond to what action is needed
        # to go to the next state.
        #
        # e.g. self.action_matrix[0][12] = 5
        self.action_matrix = np.loadtxt(path_prefix + "action_matrix.txt")

        # TODO: Fetch actions. These are the only 9 possible actions the system can take.
        # self.actions is an array of dictionaries where the row index corresponds
        # to the action number, and the value has the following form:
        # { dumbbell: "red", block: 1}
        colors = ["red", "green", "blue"]
        self.actions = np.loadtxt(path_prefix + "actions.txt")

        # TODO: fix actions mapping (it may just be a list of ints)
        self.actions = list(map(
            lambda x: {"dumbbell": colors[int(x[0])], "block": int(x[1])},
            self.actions
        ))


        # Fetch states. There are 128 states. Each row index corresponds to the
        # state number, and the value is a list of 8 items indicating whether a task 
        # has been completed (0 or 1) and a door has been closed (also 0 or 1).
        # Note: that not all states are possible to get to.
        self.states = np.loadtxt(path_prefix + "states.txt")
        self.states = list(map(lambda x: list(map(lambda y: int(y), x)), self.states))

    # Reference: https://stackoverflow.com/questions/33686747/save-a-list-to-a-txt-file
    def save_q_matrix(self, q_matrix, agent):
        # TODO: You'll want to save your q_matrix to a file once it is done to
        # avoid retraining

        # Write the Q-matrix to a .txt
        with open(os.path.dirname(__file__) + '/q_matrix_' + agent + '.txt', 'w+') as output:
            for q_matrix_row in q_matrix:
                output.write(str(list(q_matrix_row)).strip('[]') + '\n')

        return

    # Read the Q-matrix from a .txt
    def read_q_matrix(self, agent):
        # Save the Q-matrix as a 2D float array
        q_matrix_arr = []

        # Read in the .txt file
        with open(os.path.dirname(__file__) + '/q_matrix_' + agent + '.txt','r') as f:
            for line in f.readlines():
                # The line below is messy...but it converts all those string arrays in the .txt
                # to float arrays
                # (Yes, we could've just used JSON...we were devoted to .txt though)
                q_matrix_arr.append([float(x) for x in line[0:-1].split(', ')])

        return q_matrix_arr

if __name__ == "__main__":
    node = QLearning()
