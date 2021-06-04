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

### Goals
Our overarching goal was to explore Multi-Agent Reinforcement Learning (MARL) and advanced robot movement and kinematics through programming two robots to "escape" an escape room, where both robots must collaborate to complete the necessary tasks to leave the room.

In the end, we were able to program our robots to follow a pseudo-MARL policy, where they both followed a joint state / action space but weren't ultimately able to collaborate in real-time in the same world. This was due to the limitations of Gazebo; it proved to be incredibly difficult and taxing to try and recode the Gazebo worlds and related files such that two robots could operate in the world at one time. Instead, we were able to code our robots to complete their respective tasks individually in order to "escape" the room, and the tasks were determined by the MARL algorithm.

One of the interesting aspects of tackling this project is the extent to which collaboration between the robots was feasible. As mentioned previously, the limitations of Gazebo largely held us back from implementing a true, full-fledged MARL algorithm with real-time collaboration. We had to settle for a pseudo-collaborative demo, but it makes us wonder what a full collaborative implementation would look like. Exploring the kinematics of the robots and the degree to which they could pick up objects and manipulate them with the arm was also an interesting aspect of our project. We pushed the limits of the robots and had them pick up some less conventional objects, particularly with the task involving the bears. Before that, though, there was a lot of extensive trial-and-error to test exactly what the robot could and could not handle, including objects like ducks, baskets, etc. It was neat to use robot movement and kinematics with entertaining objects and tasks.


### Main Components

**Q-Learning**:
The Q-Learning component determines the optimal ordering of tasks in the escape to be completed jointly by both robots. 

Using a modified MARL algorithm, it outputs a Q-matrix that contains the order in which the tasks are completed and the doors are closed. The goal state that the Q-Learning component aims to reach is where all tasks have been completed and all doors have been closed. Doors can close before tasks in a room are completed, however, so the ordering cannot just be random.

The multi-agent Q-Learning proceeded much in the same way as the Q-Learning project. We started by creating our own state matrix and action matrix, deciding on 7 total actions (3 tasks in rooms, 3 doors on rooms + 1 exit door) and around 128 states based on these actions. The Q-matrices were then trained using these matrices, a virtual reset world, and a separate training file. Once the Q-matrices converged, they were saved as `.txt` files, ready to be read into the robot movement.


**Robot movement & kinematics**:
The robot movement was largely divided into two sections: completing room tasks and closing room doors.

Tasks: The 3 tasks that we established were hanging up bears, moving dumbbells, and "dancing". **TODO: add more detail**

Doors: There were a total of 4 doors that needed to be closed as a part of satisfying the escape room requirements. Each of the tasks occurs in a room, and for that room to be marked complete, its door must be shut. There are 3 doors that correspond to each task room, plus 1 exit door that must also close.

## System Architecture

### Matrices
We created 3 matrices, states, action-matrix and actions, all located in the action_states folder. The states matrix represents all 128 (2^7) states that our world can be in.  This was created by writing a function that computes all the possible permutations of a list, and then running it on each of the lists `[0,0,0,0,0,0,0], [1,0,0,0,0,0,0],… [1,1,1,1,1,1,1]`, to generate all possible combinations of the objects and their values. The actions matrix represents the 7 actions we are allowed to take; each of the 7 objects is allowed to be moved to its final position (represented by the value 1). This was manually written due to its simplicity. The action-matrix matrix represents each of the 128 possible states and the valid states that can be reached from that state. This was computed by first creating an empty 128x128 matrix. Then each state was compared to every other state, and if there was only one difference, and that difference represented one of the actions being completed, not undone, the location of this action was recorded. To further filter these possible actions, the action was checked against the state of the world to make sure that the action was valid at the time. For example, the yellow door could only be closed if the animals had already been moved. If these conditions weren’t true, the value was changed back to -1, if it was, this action was recorded as a valid action that could move state A to state B. This was coded through a series of loops and if/else statements and then output into the right format.


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
One of the main challenges with the Q-Learning component was the process of getting a handle on the general concepts of MARL, determining what algorithm we wanted to use, and then translating that algorithm from the research papers into our project. None of us had any experience with MARL beforehand, so it actually took a couple of days to research just to get a grip on the idea of collaborative Markov games. We then had to translate the high-level, often highly mathematical explanations into our own project, which is a much more basic implementation of MARL. The latter portion was certainly the most challenging, since we had to bridge that gap between the papers and our own work, requiring us to tweak the algorithms presented to work better for us.

**TODO: Add kinematics challenges**

## Future Work
We were happy with our project but there were definitely a lot of parts that didn’t go according to plan. A lot of our future work would be focused on fixing these issues by looking into alternate approaches and designs that might allow us to fully implement our original idea. For example, we would really like to have our robots simultaneously completing tasks in the same room. The MARL fell apart a little bit once we discovered that the robots wouldn't be able to work simultaneously in Gazebo. Our algorithm ended up exhibiting more of the single-agent Q-Learning functionality with a slight multi-agent twist, namely the joint state and action space. Additionally, future work could include making our world and objectives more complex, which would then affect the Q-Learning portion. By adding more sets of rules and additional challenges to the game, we could look into other Q-Learning algorithms that might fit this model more. There is a lot of space for us to expand on our current idea, and both fixing our issues with simultaneous robot movement and adding more components to our world would be a great place to start.

## Takeaways
- Controlling multiple robots in the same world can be very challenging 
- Integrating multiple kinematics files together requires extra launch-files, so it is good to plan ahead for how to combine all parts of the project
- Using the robot gripper with objects that aren’t very simple shapes provides a lot of extra challenges
- MARL is an incredibly large field of research, and there's a ton of different algorithsm suited to different multi-agent situations
