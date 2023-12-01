# Assignment 2 - Inverse Kinematics of DDMWR/ Sujin Park

### Problem Description
The goal of this example is to develop two cpp or python files, one called the controller and another called the driver. The driver has a topic/msg that constantly publishes the goal_pose (x,y,\theta). The controller includes two topics/msgs 1) a publisher that publishes the current pose and 2) a publisher/advertise cmd_vel. The controller implements a simple differential driver inverse kinematics of a mobile robot to determine the path of the robot. Also, implement a launch file to execute all nodes. 

###  Solution Steps
#### Driver Node
https://github.com/soojin-p/UNLV_CpE476/blob/master/assin2/src/driver.cpp
The driver has a topic/msg that constantly publishes the goal_pose (x,y,\theta).

#### Controller Node
https://github.com/soojin-p/UNLV_CpE476/blob/master/assin2/src/controller.cpp)
1. publishes the current pose
2. publishes cmd_vel


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

1. Build ROS Packages
   ``` catkin_make ```
2. Create a Launch File
```
<launch>
    <node name="driver_node" pkg="assin2_driver.cpp" type="driver" output="screen" />
    <node name="controller_node" pkg="assin2_controller.cpp" type="controller" output="screen" />
</launch>

```
3. Run Files
   1. Run the launch file
      `roslaunch assin2 assin2_launch.launch`
   2. Run each node 
   - Run the Driver Node: `rosrun assin2_driver driver_node`
   - Run the Simulator Node: `rosrun assin2_controller controller_node`

#### Screenshots and Videos
![image](https://github.com/soojin-p/UNLV_CpE476/assets/72116811/db27c17e-d7fc-4455-9faa-28023a7bafc6)
https://youtu.be/5HZYBFmda34
