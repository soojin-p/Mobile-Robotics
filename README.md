# CPE 476 - Mobile Robotics

Design, implementation, and programming of autonomous mobile robots (UAVs and Rovers), kinematics and dynamics of robots, basic control theory, sensors and actuators for robots, autopilots and autonomous control, and robot application development.

## Assignment 1: ROS - Forward Kinematics of a DDMWR / Sujin Park

### Problem Description
The goal of this assignment is to create two ROS nodes, namely the "simulator" and "driver," to simulate the forward kinematics of a differential drive mobile robot. The "driver" node publishes velocity commands to control the robot, while the "simulator" node calculates the robot's pose based on these commands. A launch file is also provided to start both nodes simultaneously.

### Solution Steps

#### Driver Node
[Driver Node Source Code](https://github.com/soojin-p/UNLV_CpE476/blob/master/assn1/src/driver.cpp)

The Driver node is responsible for controlling the robot.
1. Receives velocity commands from the `cmd_vel` topic.
2. Calculates the velocities of each wheel (Vr and Vl) based on the received commands.
3. Publishes Vr and Vl values.

#### Simulator Node
[Simulator Node Source Code](https://github.com/soojin-p/UNLV_CpE476/blob/master/assn1/src/simulator.cpp)

The Simulator node simulates the pose (position and orientation) of the robot.
1. Receives velocity commands from the `cmd_vel` topic.
2. Uses the received commands to simulate the robot's position (x, y) and orientation (theta).
3. Publishes the simulation result to the `Pose` topic.

```cpp
void cmdVelCallback(const geometry_msgs::Twist& cmd_vel) {
    double linear = cmd_vel.linear.x;
    double angular = cmd_vel.angular.z;

    double v_r = (angular * L) / 2.0 + linear;
    double v_l = linear * 2.0 - v_r;
    double R = (L / 2.0) * ((v_l + v_r) / (v_r - v_l));

    if (v_r == v_l) {
        double x_n = v_r * cos(theta) * dt;
        double y_n = v_l * sin(theta) * dt;
        x += x_n;
        y += y_n;
    } else {
        double ICC_x = x - R * sin(theta);
        double ICC_y = y + R * cos(theta);

        double omega = (v_r - v_l) / L;
        double dtheta = omega * dt;

        double x_n = cos(dtheta) * (x - ICC_x) - sin(dtheta) * (y - ICC_y) + ICC_x;
        double y_n = sin(dtheta) * (x - ICC_x) + cos(dtheta) * (y - ICC_y) + ICC_y;
        theta += dtheta;

        x = x_n;
        y = y_n;
    }
}
```

### Assignment Execution Instructions

1. **Build ROS Packages**
   ```sh
   catkin_make
   ```
2. **Create a Launch File**
```xml
<launch>
  <node name="driver_cpp_node" pkg="assn1" type="driver" output="screen" />
  <node name="simulator_cpp_node" pkg="assn1" type="simulator" output="screen" />
</launch>
```
3. **Run Files**
   1. Run the launch file:
      ```sh
      roslaunch assn1 assn1_launch.launch
      ```
   2. Run each node:
      - Run the Driver Node: `rosrun assn1_driver driver_node`
      - Run the Simulator Node: `rosrun assn1_simulator simulator_node`

#### Screenshots and Videos
![image](https://github.com/soojin-p/UNLV_CpE476/assets/72116811/08b9f845-a1b9-4c37-ad13-f138a139a856)

- [Driver and Simulator Execution Video](https://youtu.be/IDlQ4fxA2FU)

## Assignment 2: Inverse Kinematics of DDMWR / Sujin Park

### Problem Description
The goal of this assignment is to develop two cpp or python files, one called the controller and another called the driver. The driver has a topic/msg that constantly publishes the goal_pose (x, y, theta). The controller includes two topics/msgs: 1) a publisher that publishes the current pose and 2) a publisher/advertise cmd_vel. The controller implements a simple differential driver inverse kinematics of a mobile robot to determine the path of the robot. Also, implement a launch file to execute all nodes.

### Solution Steps

#### Driver Node
[Driver Node Source Code](https://github.com/soojin-p/UNLV_CpE476/blob/master/assin2/src/driver.cpp)

The driver has a topic/msg that constantly publishes the goal_pose (x, y, theta).

#### Controller Node
[Controller Node Source Code](https://github.com/soojin-p/UNLV_CpE476/blob/master/assin2/src/controller.cpp)

1. Publishes the current pose.
2. Publishes cmd_vel.

```cpp
while (ros::ok() && turtlesim_pose.x == 0.0 && turtlesim_pose.y == 0.0) {
    ros::spinOnce();
    loop_rate.sleep();
}
// Use the initial position as the starting point
goal_pose.x = turtlesim_pose.x;
goal_pose.y = turtlesim_pose.y;
goal_pose.theta = turtlesim_pose.theta;

// Loop to continuously update the goal_pose based on driver.cpp values
while (ros::ok()) {
    // Output current position for checking
    ROS_INFO("Current Position: x = %lf, y = %lf, theta = %lf", turtlesim_pose.x, turtlesim_pose.y, turtlesim_pose.theta);

    // Use goal_pose values received from driver.cpp
    moveGoal(goal_pose, 0.01);
    loop_rate.sleep();
}
```

### Assignment Execution Instructions

1. **Build ROS Packages**
   ```sh
   catkin_make
   ```
2. **Create a Launch File**
```xml
<launch>
    <node name="driver_node" pkg="assin2_driver.cpp" type="driver" output="screen" />
    <node name="controller_node" pkg="assin2_controller.cpp" type="controller" output="screen" />
</launch>
```
3. **Run Files**
   1. Run the launch file:
      ```sh
      roslaunch assin2 assin2_launch.launch
      ```
   2. Run each node:
      - Run the Driver Node: `rosrun assin2_driver driver_node`
      - Run the Simulator Node: `rosrun assin2_controller controller_node`

#### Screenshots and Videos
![image](https://github.com/soojin-p/UNLV_CpE476/assets/72116811/db27c17e-d7fc-4455-9faa-28023a7bafc6)
[Execution Video](https://youtu.be/5HZYBFmda34)

## Assignment 3: URDF

### 1) Display the robot with all TF points using Rviz
```sh
roslaunch ros_mobile_robot drive_robot.launch
```
![Screenshot from 2023-12-05 18-26-50](https://github.com/soojin-p/UNLV_CpE476/assets/72116811/d19fe6dd-e534-435e-8dab-51f6307e3511)

### 2) Move the robot using /cmd_vel using keyboard commands
```sh
rosrun rqt_robot_steering rqt_robot_steering
```
- [Robot Moving Vieo](https://github.com/soojin-p/UNLV_CpE476/issues/1#issue-2027515478)
### Additional Tasks
- **Setup the STM32CubeIDE on your PC (Linux/Windows/Mac) and validate the working of the ROS-Controller for the motors, encoders, IMU, etc. A demo was shown in the class.**
- **Calibrate the robot by completing the "09. Robot control course."**
- **Test your Lidar and Mapping by completing the "11. Lidar course - Lidar basic-SLAM."**

#### Videos and Images

#### Motors
- [Motor Video 1](https://github.com/soojin-p/UNLV_CpE476/assets/72116811/f9c6178a-a9e7-4737-839b-c3d5b0be6a87)
- [Motor Video 2](https://github.com/soojin-p/UNLV_CpE476/assets/72116811/f5ea1bb3-2cdd-42f8-822a-b25152134846)

#### Encoders
- [Encoder Video 1](https://github.com/soojin-p/UNLV_CpE476/assets/72116811/6848cb09-04b2-414e-af9a-1a4c411d39b0)
- [Encoder Video 2](https://github.com/soojin-p/UNLV_CpE476/assets/72116811/82c4ea84-b2c6-4efd-8692-6290503cedc9)
- [Encoder Video 3](https://youtube.com/shorts/WA-CgaBt0xM?feature=share)

#### IMU
- [IMU Video 1](https://github.com/soojin-p/UNLV_CpE476/assets/72116811/ef1af55c-88be-4b3d-a9b4-a14dc4428435)
- [IMU Video 2](https://github.com/soojin-p/UNLV_CpE476/assets/72116811/2a540f09-f313-4de7-a017-792df1506bed)
- [IMU Video 3](https://youtube.com/shorts/8CiImidPCtI?feature=share)

#### Calibration
- [Calibration Video](https://youtube.com/shorts/B6WupWhTcLo?feature=share)

#### Jetson Nano Setup
- ![Jetson Nano Setup 1](https://github.com/soojin-p/UNLV_CpE476/assets/72116811/bc760156-33fe-435e-b714-0a20e0c46bf3)
- ![Jetson Nano Setup 2](https://github.com/soojin-p/UNLV_CpE476/assets/72116811/78f911f8-6ea2-41e3-a37d-518c175e49ed)

#### Mapping
- ![Mapping Screenshot 1](https://github.com/soojin-p/UNLV_CpE476/assets/72116811/4a6c182d-c43e-47e1-bfdc-154ae2b85ce0)
- ![Mapping Screenshot 2](https://github.com/soojin-p/UNLV_CpE476/assets/72116811/a9c6b170-bd99-4a62-979b-1dd85e902d09)
- ![Mapping Screenshot 3](https://github.com/soojin-p/UNLV_CpE476/assets/72116811/89eae955-3853-4736-b6c3-2e554ea4c107)
- [Mapping Video](https://youtu.be/hyZjB1GkSXk)

## References
- [Raspberry Pi4 Camera](https://www.waveshare.com/wiki/RPi_Camera_%28I%29#libcamera-still)
- [ESP32-E](https://wiki.dfrobot.com/FireBeetle_Board_ESP32_E_SKU_DFR0654)
- [OpenCV Shadow Remover](https://github.com/YalimD/image_shadow_remover)

## Code Repositories
- **ESP32-E code:** [ESP32_Shadow_Remove](https://github.com/soojin-p/UNLV_Project/tree/main/ESP32_Shadow_Remove)
- **Raspberry Pi4 Camera code:** [RPi Camera](https://github.com/soojin-p/UNLV_Project/blob/main/RPi%20Camera.md)
- **OpenCV code:** [OpenCV_Shadow_detect](https://github.com/soojin-p/UNLV_Project/tree/main/OpenCV_Shadow_detect)

## Conclusion
This collection of assignments and projects showcases the design, implementation, and programming of autonomous mobile robots, emphasizing the use of ROS for robot control and navigation. Through these exercises, students gain hands-on experience with kinematics, dynamics, control theory, sensor integration, and autonomous systems development.
