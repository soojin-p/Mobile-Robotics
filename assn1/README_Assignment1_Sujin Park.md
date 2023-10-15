# Assignment1: ROS - Forward Kinematics of a DDMWR / Sujin park

### Problem Description
The goal of this assignment is to create two ROS nodes, namely the "simulator" and "driver," to simulate the forward kinematics of a differential drive mobile robot. The "driver" node publishes velocity commands to control the robot, while the "simulator" node calculates the robot's pose based on these commands. A launch file is also provided to start both nodes simultaneously.
###  Solution Steps
#### Driver Node: https://github.com/soojin-p/UNLV_CpE476/blob/master/assn1/src/driver.cpp
The Driver node is responsible for controlling the robot.
1. Receives velocity commands from the `cmd_vel` topic.
2. Calculates the velocities of each wheel (Vr and Vl) based on the received commands.
3. Publishes Vr and Vl values.
   
#### Simulator Node: https://github.com/soojin-p/UNLV_CpE476/blob/master/assn1/src/simulator.cpp)
The Simulator node simulates the pose (position and orientation) of the robot. 
1. Receives velocity commands from the `cmd_vel` topic.
2. Uses the received commands to simulate the robot's position (x, y) and orientation (theta).
3. Publishes the simulation result to the `Pose` topic.
```cpp
void cmdVelCallback(const geometry_msgs::Twist& cmd_vel) {

    double linear = cmd_vel.linear.x;
    double angular = cmd_vel.angular.z;

  // Calculate velocities of the right and left wheels
    double v_r = (angular * L) / 2.0 + linear;
    double v_l = linear * 2.0 - v_r;
    double R = (L / 2.0) * ((v_l + v_r) / (v_r - v_l));

    if (v_r == v_l) {
        // The robot is moving straight, no rotation
        double x_n = v_r * cos(theta) * dt;
        double y_n = v_l * sin(theta) * dt;
        x += x_n;
        y += y_n;
    }
    else {
        // The robot is turning, so calculate the changes in orientation, x, and y

        double ICC_x = x - R * sin(theta);
        double ICC_y = y + R * cos(theta);

        double omega = (v_r - v_l) / L;
        double dtheta = omega * dt;

        double x_n = cos(dtheta) * (x - ICC_x) - sin(dtheta) * (y - ICC_y) + ICC_x;
        double y_n = sin(dtheta) * (x - ICC_x) - cos(dtheta) * (y - ICC_y) + ICC_y;
        theta += dtheta;

        x = x_n;
        y = y_n;
    }
}
```
### Assignment Execution Instructions

1. Build ROS Packages
   ``` catkin_make ```
2. Create a Launch File
```
<launch>
  <node name="driver_cpp_node" pkg="assn1" type="driver" output="screen" />
  <node name="simulator_cpp_node" pkg="assn1" type="simulator" output="screen"  />
<launch>
```
3. Run Files
   1. Run the launch file
      `roslaunch assn1 assn1_launch.launch`
   2. Run each node 
   - Run the Driver Node: `rosrun my_robot_driver driver_node`
   - Run the Simulator Node: `rosrun my_robot_simulator simulator_node`

#### Screenshots and Videos
![image](https://github.com/soojin-p/UNLV_CpE476/assets/72116811/08b9f845-a1b9-4c37-ad13-f138a139a856)


- [Driver and Simulator Execution Video Link](https://youtu.be/07lgEeBNV0I)
