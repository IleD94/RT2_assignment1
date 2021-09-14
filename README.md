# First Assignment of the Research Track 2 course - action branch


##General
This package contains the almost the same file of the main branch. The difference is that we use an action to control the robot. For this reason we have had chage the go_to_point.py file, that in this branch is **go_to_point_action.py**, changing properly to take into account the mechanism of the action. Due to that, we have had changed also **state_machine.cpp** in such way we can set and cancel the goal using this node.

#PlanningAction

The structure of the Planning action is the following:
```
geometry_msgs/PoseStamped target_pose
---
bool result
---
geometry_msgs/Pose actual_pose
string stat
```
