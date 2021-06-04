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

### Code Location

## Challenges

## Future Work

## Takeaways