# Escapebots

## Contributors
- Stephanie Kim
- Lucy Li
- Josephine Passananti
- Zhou Xing

## Motivation

We will explore the collaboration/competition of two robots to finish tasks in a room in a specific order as fast as possible, so that they will be let out of the room. Each task can only be finished by one robot. This will involve some fun designs of the room, different customized tasks, and also the interaction of robots.

## Instruction (demo)

0. `git clone` or `git pull`, then `cd ~/catkin_ws && catkin_make` 
   
1. In terminal: **(Don't forget to `cd ~/catkin_ws && catkin_make` for each new terminal window)**

```
(1st window) $ roscore
(2st window) $ roslaunch escapebots sing_demo.launch
(3nd window) $ roslaunch turtlebot3_manipulation_moveit_config move_group.launch
(Wait until you see "You can start planning now!!")
(4th window) $ rosrun escapebots demopickup.py
```

## Project Description

### Goal

### Main Components

## System Architecture

### Algorithm
The main robotics algorithm we were looking to implement is Nash Q-Learning, which is a kind of Multi-Agent Reinforcement Learning (MARL) algorithm. Nash Q-Learning works by establishing a Nash equilibrium strategy among all learning agents such that each agent's actions are the best response to any other agent's action. 

We wanted to implement a fully cooperative version of Nash Q-Learning, where all robots intend to maximize the common reward. To do so, we used a common fully cooperative multi-agent Q-Learning implementation, wherein each robot maintains its own Q-function and Q-matrix. In the update for each robot's Q-function, it differs slightly from single-agent Q-Learning in that the robot accounts for the other robot's Q-values and Nash equilibrium strategy. The general equation is as follows:

`Q(s, a1, ... ,aN) = (1 - alpha) * Q(s, a1) + alpha * (reward + gamma * Nash(s', q1, ..., qN))`

where alpha = learning rate, gamma = discount factor, and aN = the Q-value of robot N at state s.

We used a modified version of the Nash Q-value (Nash(s', a1, ..., aN)) since our robots are fully collaborative. We set `Nash(s', a1, ..., aN))` equal to the maximum Q-value derived across all learning agents -- in our case, since we only have two robots, the maximum Q-value across both robots. This new Nash Q-value is a slight spinoff of the Q-value used in the Minimax algorithm, which is another Q-Learning algorithm for competitive MARL. 

Since we use a learning rate of 1, the algorithm distills down to a much simpler version that resembles single-agent Q-Learning. This should be the case, as the MARL we use should be somewhat similar to the single-agent learning given that our robots must be fully cooperative.

### Code Location
Essentially all of the code for our MARL algorithm and functions can be found in the `q_learning_training.py`. The `update_q_matrix()` function is particularly important, as it is the epicenter of the algorithm's execution.

## Challenges

## Future Work

## Takeaways