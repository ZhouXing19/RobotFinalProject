import numpy as np
import rospy, sys, os

# Path of directory on where this file is located
# path_prefix = os.path.dirname(__file__) + "/action_states/"
print("starting")

class Actions(object):

    def __init__(self):

        rospy.init_node("Actions")
        rospy.sleep(1)
        print("initialized node")

        self.action_matrix = np.loadtxt(path_prefix + "/action_states/action_matrix.txt")

        # Import qmatrix, action matrix and actions
        # import qmatrix
        self.qmatrix = []

        # create an empty action list
        self.action_list = []

        self.run()
    
    def read_q_matrix(self, agent):
        # Save the Q-matrix as a 2D float array
        q_matrix_arr = []

        # Read in the .txt file
        with open(os.path.dirname(__file__) + 'q_matrix_' + agent + '.txt','r') as f:
            for line in f.readlines():
                # The code below is messy...
                # ...but it converts all those string arrays in the .txt to float arrays
                # (We could've just used JSON. We were devoted to .txt though)
                q_matrix_arr.append([float(x) for x in line[0:-1].split(', ')])

        self.qmatrix = q_matrix_arr


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

        print(self.action_list)

    def run(self):
        self.read_q_matrix('a')
        self.read_matrix()
            
# run our node and keep rospy spinnning
if __name__ == "__main__":
    node = Actions()
    rospy.spin()
