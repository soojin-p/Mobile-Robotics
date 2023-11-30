#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "driver_node");
    ros::NodeHandle nh;

    ros::Publisher pose_publisher = nh.advertise<geometry_msgs::Pose2D>("goal_pose", 1);

    ros::Rate loop_rate(1);

    while (ros::ok()) {
        geometry_msgs::Pose2D goal_pose;

        goal_pose.x = 10.0; 
        goal_pose.y = 7.0;
        goal_pose.theta = 0.0;

        pose_publisher.publish(goal_pose);
        ROS_INFO("Published Goal Pose: x = %lf, y = %lf, theta = %lf", goal_pose.x, goal_pose.y, goal_pose.theta);

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
