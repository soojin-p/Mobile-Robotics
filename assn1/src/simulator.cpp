//simulator
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
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