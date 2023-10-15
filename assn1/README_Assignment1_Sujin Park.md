# Assignment1: ROS - Forward Kinematics of a DDMWR / Sujin park

### Problem Description
The goal of this assignment is to create two ROS nodes, namely the "simulator" and "driver," to simulate the forward kinematics of a differential drive mobile robot. The "driver" node publishes velocity commands to control the robot, while the "simulator" node calculates the robot's pose based on these commands. A launch file is also provided to start both nodes simultaneously.
###  Solution Steps
#### [Driver Node](https://github.com/soojin-p/UNLV_CpE476/blob/master/assn1/src/driver.cpp)
The Driver node is responsible for controlling the robot.
1. Receives velocity commands from the `cmd_vel` topic.
2. Calculates the velocities of each wheel (Vr and Vl) based on the received commands.
3. Publishes Vr and Vl values.
```cpp
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>

std_msgs::Float64 g_Vr;
std_msgs::Float64 g_Vl;

void cmdVelCallback(const geometry_msgs::Twist& cmd_vel) {

	double linear = cmd_vel.linear.x; // Linear velocity
	double angular = cmd_vel.angular.z; // Linear velocity

	double R = 1.0; // Radius of the wheels
	double L = 4.0; // Distance between the wheels

	// Calculate right and left wheel velocities
	double v_r = ((linear + (angular * L / 2.0)) / R);
	double v_l = ((linear - (angular * L / 2.0)) / R);

	g_Vr.data = v_r;
	g_Vl.data = v_l;
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "driver_node");
	ros::NodeHandle nh;

	ros::Subscriber cmd_vel_sub = nh.subscribe("cmd_vel", 1, cmdVelCallback);

	ros::Publisher Vr_pub = nh.advertise<std_msgs::Float64>("Vr", 1);
	ros::Publisher Vl_pub = nh.advertise<std_msgs::Float64>("Vl", 1);

	double dt_controller = 0.01;
	double sample_rate = 1.0 / dt_controller; // compute the corresponding update frequency 
	ros::Rate naptime(sample_rate); // use to regulate loop rate 

	while (ros::ok()) {
		// Publish the right and left wheel velocities
		Vr_pub.publish(g_Vr);
		Vl_pub.publish(g_Vl);

		ROS_INFO("V_r = %f", g_Vr.data);
		ROS_INFO("V_l = %f", g_Vl.data);

		ros::spinOnce();
		naptime.sleep();
	}
	return 0;
}
```
   
#### [Simulator Node](https://github.com/soojin-p/UNLV_CpE476/blob/master/assn1/src/simulator.cpp)
The Simulator node simulates the pose (position and orientation) of the robot. 
1. Receives velocity commands from the `cmd_vel` topic.
2. Uses the received commands to simulate the robot's position (x, y) and orientation (theta).
3. Publishes the simulation result to the `Pose` topic.
```cpp
//simulator
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>

// Initialize variables for the robot's position and orientation
double x = 0.0;
double y = 0.0;
double theta = 0.0;
double dt = 0.01;

void cmdVelCallback(const geometry_msgs::Twist& cmd_vel) {
   
    double velocity = cmd_vel.linear.x;
    double angular = cmd_vel.angular.z;

    double delta_theta = angular * dt;

    // Calculate changes in orientation, x, and y based on velocity commands
    double delta_x = velocity * cos(theta + delta_theta / 2) * dt;
    double delta_y = velocity * sin(theta + delta_theta / 2) * dt;

    x += delta_x;
    y += delta_y;
    theta += delta_theta;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "simulator");
    ros::NodeHandle nh;

    ros::Subscriber cmd_vel_sub = nh.subscribe("cmd_vel", 1, cmdVelCallback);
    ros::Publisher pose_pub = nh.advertise<geometry_msgs::Pose>("Pose", 1);
 
    double sample_rate = 1.0 / dt;
    ros::Rate naptime(sample_rate);

    while (ros::ok()) {

        // Create a message for the current robot pose
        geometry_msgs::Pose pose;
        pose.position.x = x;
        pose.position.y = y;

        pose_pub.publish(pose);

        // Print the current position 
        ROS_INFO("pose.x = %f", pose.position.x);
        ROS_INFO("pose.y = %f", pose.position.y);
        ros::spinOnce();
        naptime.sleep();
    }
    return 0;
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
![image](https://github.com/soojin-p/UNLV_CpE476/assets/72116811/31a92f6c-df5d-4b9f-a137-b8b8a4744a35)
![image](https://github.com/soojin-p/UNLV_CpE476/assets/72116811/216d2460-1cab-496d-b0f3-62129256b5a9)

- [Driver and Simulator Execution Video Link](https://youtu.be/07lgEeBNV0I)
