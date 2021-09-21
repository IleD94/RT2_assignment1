# First Assignment of the Research Track 2 course - main branch

The package contains the nodes and the simulation environment for controlling a mobile robot in the Gazebo simulation environment.
To launch the node, please run:
```
roslaunch rt2_assignment1 sim.launch
```
# Scene in Coppelia environment:

The file: **myAssignmentScene.ttt**  contains a scene in the Coppelliasim simulator environment for ROS. There we have a robot_pioneer_ctrl that moves to reach the random goal that receives from the other nodes of the branch. This communication with the robot in the simulator is let thanks a pubisher to the /odom topic , that is declared in the LUA script of the robot, the same topic is subscribed by the go_to_point node, in this way the two nodes communicate between each other. Moreover there is also a subscriber to /cmd_vel topic that receive the messages from the go_to_point publisher to the same topic.

## How to run the scene?

Please follows this steps to run the Vrep scene:
1. Open a terminal and run the master, using the command:
```
roscore&
```
2. In the same terminal run Coppelia with the command:
```
./coppeliasim.sh
```
3. Open the scene: myAssignmentScene.ttt in Coppelia
4. Open one more terminal and launch the file that contains the other 4 nodes:
```
roslaunch rt2_assignment1 VrepSim.launch
```
