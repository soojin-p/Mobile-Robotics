#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "turtlesim/Pose.h"
#include <geometry_msgs/Pose2D.h>
#include <sstream>

using namespace std;

ros::Publisher velocity_publisher;
ros::Subscriber pose_subscriber;
ros::Subscriber goal_pose_subscriber;
turtlesim::Pose turtlesim_pose;

void move(double speed, double distance, bool isForward);
void poseCallback(const turtlesim::Pose::ConstPtr& pose_message);
void goalPoseCallback(const geometry_msgs::Pose2D::ConstPtr& goal_pose_msg);
void moveGoal(turtlesim::Pose goal_pose, double distance_tolerance);
double getDistance(double x1, double y1, double x2, double y2);

int main(int argc, char** argv)
{
    ros::init(argc, argv, "turtlesim_cleaner");
    ros::NodeHandle n;

    velocity_publisher = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);
    pose_subscriber = n.subscribe("/turtle1/pose", 10, poseCallback);
    goal_pose_subscriber = n.subscribe("/goal_pose", 10, goalPoseCallback);
    ros::Rate loop_rate(0.5);

    ROS_INFO("\n\n\n ********START TESTING*********\n");

    while (ros::ok()) {
        // Process ROS events
        ros::spinOnce();

        // Output current position for checking
        ROS_INFO("Current Position: x = %lf, y = %lf, theta = %lf", turtlesim_pose.x, turtlesim_pose.y, turtlesim_pose.theta);

        turtlesim::Pose goal_pose;  // goal_pose를 미리 정의
        moveGoal(goal_pose, 0.01);
        loop_rate.sleep();
    }

    return 0;
}

void move(double speed, double distance, bool isForward) {
    geometry_msgs::Twist vel_msg;

    if (isForward)
        vel_msg.linear.x = abs(speed);
    else
        vel_msg.linear.x = -abs(speed);

    vel_msg.linear.y = 0;
    vel_msg.linear.z = 0;
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
        loop_rate.sleep();
    } while (current_distance < distance);

    vel_msg.linear.x = 0;
    velocity_publisher.publish(vel_msg);
}

void poseCallback(const turtlesim::Pose::ConstPtr& pose_message) {
    turtlesim_pose.x = pose_message->x;
    turtlesim_pose.y = pose_message->y;
    turtlesim_pose.theta = pose_message->theta;
    ROS_INFO("Current Position: x = %lf, y = %lf, theta = %lf", turtlesim_pose.x, turtlesim_pose.y, turtlesim_pose.theta);
}

void goalPoseCallback(const geometry_msgs::Pose2D::ConstPtr& goal_pose_msg) {
    turtlesim_pose.x = goal_pose_msg->x;
    turtlesim_pose.y = goal_pose_msg->y;
    turtlesim_pose.theta = goal_pose_msg->theta;
    ROS_INFO("Goal Position: x = %lf, y = %lf, theta = %lf", turtlesim_pose.x, turtlesim_pose.y, turtlesim_pose.theta);


}

void moveGoal(turtlesim::Pose goal_pose, double distance_tolerance) {
    geometry_msgs::Twist vel_msg;
    ros::Rate loop_rate(10);

    do {
        vel_msg.linear.x = 1.5 * getDistance(turtlesim_pose.x, turtlesim_pose.y, goal_pose.x, goal_pose.y);
        vel_msg.linear.y = 0;
        vel_msg.linear.z = 0;
        vel_msg.angular.x = 0;
        vel_msg.angular.y = 0;
        vel_msg.angular.z = 2 * (atan2(goal_pose.y - turtlesim_pose.y, goal_pose.x - turtlesim_pose.x) - turtlesim_pose.theta);

        velocity_publisher.publish(vel_msg);
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
