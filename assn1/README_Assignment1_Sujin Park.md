# Assignment1: ROS - Forward Kinematics of a DDMWR / Sujin park

### Problem Description
The goal of this assignment is to create two ROS nodes, namely the "simulator" and "driver," to simulate the forward kinematics of a differential drive mobile robot. The "driver" node publishes velocity commands to control the robot, while the "simulator" node calculates the robot's pose based on these commands. A launch file is also provided to start both nodes simultaneously.
###  Solution Steps
#### Driver Node
The Driver node is responsible for controlling the robot.
- [Driver Node Code Link](link_to_driver_code)
1. Receives velocity commands from the `cmd_vel` topic.
2. Calculates the velocities of each wheel (Vr and Vl) based on the received commands.
3. Publishes Vr and Vl values.
   
#### Simulator Node
The Simulator node simulates the pose (position and orientation) of the robot. 
- [Simulator Node Code Link](link_to_simulator_code)
1. Receives velocity commands from the `cmd_vel` topic.
2. Uses the received commands to simulate the robot's position (x, y) and orientation (theta).
3. Publishes the simulation result to the `Pose` topic.

### Assignment Execution Instructions

1. Build ROS Packages
   ``` catkin_make ```
2. Create a Launch File
```
<launch>
  <node name="driver_cpp_node" pkg="assn1" type="driver" />
  <node name="simulator_cpp_node" pkg="assn1" type="simulator" />
<launch>
```
3. Run Files
   1. Run the launch file: `roslaunch assn1 assn1_launch.launch`
   2. Run each node 
   - Run the Driver Node: `rosrun my_robot_driver driver_node`
   - Run the Simulator Node: `rosrun my_robot_simulator simulator_node`

#### Videos
- [Driver and Simulator Execution Video Link](link_to_driver_video)
