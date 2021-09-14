# First Assignment of the Research Track 2 course - action branch


##General
This package contains the almost the same file of the main branch. The difference is that we use an action to control the robot. For this reason we have had chage the go_to_point.py file, that in this branch is **go_to_point_action.py**, changing properly to take into account the mechanism of the action. Due to that, we have had changed also **state_machine.cpp** in such way we can set and cancel the goal using this node.

# PlanningAction

The structure of the Planning action that we use to this scope is the following:
```
geometry_msgs/PoseStamped target_pose
---
bool result
---
geometry_msgs/Pose actual_pose
string stat
```
The first field corresponds to the goal, the second is the result and the last on is the feedback.

# How to run the package?

In order to run this package, follow this simple steps

1. Open a terminal
2. Type the command below:
```
roslaunch rt2_assignment1 sim.launch
```
3. Follow the instruction on the bash to start or stop the robot
