#include <ros/ros.h>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include "geometry_msgs/Pose2D.h" 
#include <sstream>
#include <iostream>


using namespace std;

ros::Publisher velocity_publisher;
ros::Subscriber pose_subscriber;	// to determine the position for turning the robot in an absolute orientation --> in the setDesiredOrientation fn
ros::Subscriber goal_pose_subscriber;  // Subscriber for the goal_pose topic
turtlesim::Pose turtlesim_pose;


void move(double speed, double distance, bool isForward);
void poseCallback(const turtlesim::Pose::ConstPtr& pose_message);	//Callback fn everytime the turtle pose msg is published over the /turtle1/pose topic.
void moveGoal(turtlesim::Pose goal_pose, double distance_tolerance);	//this will move robot to goal
double getDistance(double x1, double y1, double x2, double y2);



int main(int argc, char** argv)
{
	// Initiate new ROS node named "talker"
	ros::init(argc, argv, "turtlesim_cleaner");
	ros::NodeHandle n;

	velocity_publisher = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);
	pose_subscriber = n.subscribe("/turtle1/pose", 10, poseCallback);	//call poseCallback everytime the turtle pose msg is published over the /turtle1/pose topic.
	goal_pose_subscriber = n.subscribe("/goal_pose", 10, goalPoseCallback);
	ros::Rate loop_rate(0.5);


	ROS_INFO("\n\n\n ********START TESTING*********\n");

	turtlesim::Pose goal_pose;

	// Waiting for the first pose message to arrive and set the initial position
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

	return 0;
}
void move(double speed, double distance, bool isForward) {
	geometry_msgs::Twist vel_msg;
	//set a random linear velocity in the x-axis
	if (isForward)
		vel_msg.linear.x = abs(speed);
	else
		vel_msg.linear.x = -abs(speed);
	vel_msg.linear.y = 0;
	vel_msg.linear.z = 0;
	//set a random angular velocity in the y-axis
	vel_msg.angular.x = 0;
	vel_msg.angular.y = 0;
	vel_msg.angular.z = 0;

	double t0 = ros::Time::now().toSec();
	double current_distance = 0.0;
	ros::Rate loop_rate(100);
	do {
		velocity_publisher.publish(vel_msg);
		double t1 = ros::Time::now().toSec();
		current_distance = speed * (t1 - t0);
		ros::spinOnce();
		loop_rate.sleep();
		//cout<<(t1-t0)<<", "<<current_distance <<", "<<distance<<endl;
	} while (current_distance < distance);
	vel_msg.linear.x = 0;
	velocity_publisher.publish(vel_msg);

}

void poseCallback(const turtlesim::Pose::ConstPtr& pose_message) {
	turtlesim_pose.x = pose_message->x;
	turtlesim_pose.y = pose_message->y;
	turtlesim_pose.theta = pose_message->theta;
}


void moveGoal(turtlesim::Pose goal_pose, double distance_tolerance) {
	//We implement a Proportional Controller. We need to go from (x,y) to (x',y'). Then, linear velocity v' = K ((x'-x)^2 + (y'-y)^2)^0.5 where K is the constant and ((x'-x)^2 + (y'-y)^2)^0.5 is the Euclidian distance. The steering angle theta = tan^-1(y'-y)/(x'-x) is the angle between these 2 points.
	geometry_msgs::Twist vel_msg;

	ros::Rate loop_rate(10);
	do {
		//linear velocity
		vel_msg.linear.x = 1.5 * getDistance(turtlesim_pose.x, turtlesim_pose.y, goal_pose.x, goal_pose.y);
		vel_msg.linear.y = 0;
		vel_msg.linear.z = 0;
		//angular velocity
		vel_msg.angular.x = 0;
		vel_msg.angular.y = 0;
		vel_msg.angular.z = 2 * (atan2(goal_pose.y - turtlesim_pose.y, goal_pose.x - turtlesim_pose.x) - turtlesim_pose.theta);

		velocity_publisher.publish(vel_msg);

		ros::spinOnce();
		loop_rate.sleep();

	} while (getDistance(turtlesim_pose.x, turtlesim_pose.y, goal_pose.x, goal_pose.y) > distance_tolerance);
	cout << "end move goal" << endl;
	vel_msg.linear.x = 0;
	vel_msg.angular.z = 0;
	velocity_publisher.publish(vel_msg);

}

double getDistance(double x1, double y1, double x2, double y2) {
	return sqrt(pow((x2 - x1), 2) + pow((y2 - y1), 2));
}

void goalPoseCallback(const geometry_msgs::Pose2D::ConstPtr& goal_pose_msg)
{
	goal_pose.x = goal_pose_msg->x;
	goal_pose.y = goal_pose_msg->y;
	goal_pose.theta = goal_pose_msg->theta;
}