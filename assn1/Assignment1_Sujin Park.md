# Assignment1: ROS - Forward Kinematics of a DDMWR / Sujin park
### Problem Understanding and Solution Steps
#### Driver Node
The Driver node is responsible for controlling the robot.
1. Receives velocity commands from the `cmd_vel` topic.
2. Calculates the velocities of each wheel (Vr and Vl) based on the received commands.
3. Publishes Vr and Vl values to the `Vr` and `Vl` topics.
   
#### Simulator Node
The Simulator node simulates the pose (position and orientation) of the robot. 
1. Receives velocity commands from the `cmd_vel` topic.
2. Uses the received commands to simulate the robot's position (x, y) and orientation (theta).
3. Publishes the simulation result to the `Pose` topic.

### Code Description and Code Links
- [Driver Node Code Link](link_to_driver_code)
- [Simulator Node Code Link](link_to_simulator_code)

### Assignment Execution Instructions

1. Build ROS Packages
   ``` catkin_make '''
2. Create a Launch File
   ```
<launch>
  <node name="driver_cpp_node" pkg="assn1" type="driver" />
  <node name="simulator_cpp_node" pkg="assn1" type="simulator" />
</launch>
  ```
3. Run Files
   - Run the launch file to start the "driver" and "simulator" nodes.: `roslaunch assn1 assn1_launch.launch`
   - Run the Driver Node: `rosrun my_robot_driver driver_node`
   - Run the Simulator Node: `rosrun my_robot_simulator simulator_node`

#### Videos
- [Driver and Simulator Execution Video Link](link_to_driver_video)
