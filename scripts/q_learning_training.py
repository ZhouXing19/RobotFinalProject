#!/usr/bin/env python3

import rospy
import numpy as np
import random
from q_learning import QLearning
from escapebots.msg import QLearningReward, QMatrix, QMatrixRow, RobotTasksDoors

class QLearningTraining(object):
    # Initialize publishers and subscribers
    def __init__(self):
        # Publish Q-matrix updates to Q-matrix topic
        self.q_matrix_pub = rospy.Publisher("/RobotFinalProject/QMatrix", QMatrix, queue_size=10)

        # Publish to /robot_action for training the Q-matrices
        self.q_learning = QLearning()
        self.robot_action_pub = rospy.Publisher("/RobotFinalProject/robot_action", RobotTasksDoors, queue_size=10)

        # Subscribe to environment to receive reward updates
        self.reward = rospy.Subscriber("/RobotFinalProject/reward", QLearningReward, self.update_q_matrix)
        rospy.sleep(2)

        # Initialize Q-matrices, one for each robot
        self.q_matrix_msg_a = QMatrix()
        self.q_matrix_msg_b = QMatrix()

        # TODO: For ease of use in training, we'll store the Q-matrices in a numpy array
        self.q_matrix_arr_a = np.zeros((64,9))
        self.q_matrix_arr_b = np.zeros((64,9))

        # Keep track of the 5 most recently updated Q-values per Q-matrix
        self.q_history_a = []
        self.q_history_b = []
        
        # Keeps track of current (starting) state, initialized to 0th state. This state accounts for both robots
        self.current_state = 0
        
        # Get new state and action from starting state
        new_state, action = self.get_random_action(self.current_state)

        # Current action is action from above (action from current state to new state)
        self.current_action = action

        # Keep track of new state
        self.new_state = new_state

        # TODO: Move the robot according to the current action
        

    # Selects a random valid action from the current state
    def get_random_action(self, state):
        # Stores all valid actions at the current state
        valid_actions = []

        # Search each next possible state from the current state
        for (s, a) in enumerate(self.q_learning.action_matrix[state]):
            # If valid, append to valid_actions
            if int(a) != -1:
                valid_actions.append((s, int(a)))

        # Select a random action among the valid actions
        if len(valid_actions) > 0:
            (new_state, action) = random.choice(valid_actions)
            return (new_state, action)
        else: # Otherwise, reset to the initial state
            self.current_state = 0
            s_a_from_origin = self.get_random_action(0)
            return s_a_from_origin

    def update_q_matrix(self, data):
        # data.reward receives the reward
        # TODO: same reward between both robots?

        # Discount factor
        gamma = 0.9

        # Learning rate
        alpha = 1.0

        # Update Q(s,a) for each robot
        # 2 Q-functions, 1 for each learning agent (robot), each dependent on actions of both robots
        # Full equation: Q_x = (1 - alpha) * Q_x + alpha * (reward + gamma * Nash(s, a, b))
        # 1 - alpha * Q_x = 0 when alpha = 1, so that is taken out below
        Q_a = alpha * (data.reward + gamma * max(self.q_matrix_arr_a[self.new_state]) * max(self.q_matrix_arr_b[self.new_state]))
        Q_b = alpha * (data.reward + gamma * max(self.q_matrix_arr_b[self.new_state]) * max(self.q_matrix_arr_a[self.new_state]))


        old_q_a = self.q_matrix_arr_a[self.current_state, self.current_action]
        old_q_b = self.q_matrix_arr_b[self.current_state, self.current_action]

        # Append the change in Q-value to q_history to see whether Q-value changes are plateauing
        self.q_history_a.append(Q_a - old_q_a)
        self.q_history_b.append(Q_b - old_q_b)
        
        # Update the Q-matrix in the current spot with the new Q-value
        self.q_matrix_arr_a[self.current_state, self.current_action] = Q_a
        self.q_matrix_arr_b[self.current_state, self.current_action] = Q_b
        
        # We need to convert the numpy Q-matrix used for testing back into a QMatrix() message type
        self.convert_send_qmatrix_msg()

        # If not converged:
        if not self.has_converged():
            # Update the current state
            self.current_state = self.new_state

            # Select the next action
            new_state, action = self.get_random_action(self.current_state)

            # Update the new state
            self.new_state = new_state
            self.current_action = action

            # TODO: Perform the next action
            return

    # Determines when the Q-matrix has converged
    def has_converged(self):
        # Establish a plateau threshold, i.e. a constant below which the q-value differences should fall in order to
        # qualify the changes as plateauing
        max_diff = 0.01

        # Algo has converged if 
        # 1) the last 100 q-value differences are below the plateau threshold
        # 2) we have iterated at least 10000 times 
        # TODO: debugging
        if len(self.q_history_a) > 10000 and max(self.q_history_a[-100:]) < max_diff:
            self.q_learning.save_q_matrix(self.q_matrix_arr_a, "a")
            print("A Converged!")

        if len(self.q_history_b) > 10000 and max(self.q_history_b[-100:]) < max_diff:
            self.q_learning.save_q_matrix(self.q_matrix_arr_b, "b")
            print("B Converged! You may Ctrl+C")
            return True
        
        return False
    
    def convert_send_qmatrix_msg(self): # Converts numpy array to QMatrix msg
        self.q_matrix_msg_a = QMatrix()
        self.q_matrix_msg_b = QMatrix()
        for i in range(len(self.q_matrix_arr_a)):
            row = QMatrixRow()
            row.q_matrix_row = self.q_matrix_arr_a[i].astype(int)
            self.q_matrix_msg_a.q_matrix.append(row)

        for i in range(len(self.q_matrix_arr_b)):
            row = QMatrixRow()
            row.q_matrix_row = self.q_matrix_arr_b[i].astype(int)
            self.q_matrix_msg_b.q_matrix.append(row)

        # Publish Q-matrix messages to Q-matrix topic
        self.q_matrix_pub.publish(self.q_matrix_msg_a)
        self.q_matrix_pub.publish(self.q_matrix_msg_b)

    # Runs until shutdown
    def run(self):
        rospy.spin()

# Runs file
if __name__ == "__main__":
    training_node = QLearningTraining()
    training_node.run()