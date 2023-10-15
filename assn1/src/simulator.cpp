#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <cmath>

double L = 4.0; // Distance between the wheels
double x = 0.0;
double y = 0.0;
double theta = 0.0;
double dt = 0.01; // Time step 

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

int main(int argc, char** argv) {
    ros::init(argc, argv, "simulator");
    ros::NodeHandle nh;

    ros::Subscriber cmd_vel_sub = nh.subscribe("cmd_vel", 1, cmdVelCallback);
    ros::Publisher pose_pub = nh.advertise<geometry_msgs::Pose2D>("Pose", 1);
    ros::Time ctime, ptime;
    ctime = ros::Time::now();
    ptime = ros::Time::now();

    double sample_rate = 1.0 / dt;
    ros::Rate naptime(sample_rate);

    while (ros::ok()) {

        ctime = ros::Time::now();
        dt = (ctime - ptime).toSec();


        // Create a message for the current robot pose
        geometry_msgs::Pose2D pose;
        pose.x = x;
        pose.y = y;
        pose.theta = theta;

        pose_pub.publish(pose);

        ROS_INFO("pose.x = %f", pose.x);
        ROS_INFO("pose.y = %f", pose.y);
        ros::spinOnce();
        naptime.sleep();
    }
    return 0;
}
