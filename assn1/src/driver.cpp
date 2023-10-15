#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "driver_node");
    ros::NodeHandle nh;

    ros::Publisher cmd_vel_publisher = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    double dt_controller = 0.01;
    double sample_rate = 1.0 / dt_controller; // compute the corresponding update frequency
    ros::Rate naptime(sample_rate); // use to regulate loop rate

    while (ros::ok()) {
        geometry_msgs::Twist cmd_vel; // Create a Twist message

        // Set linear and angular velocities in the cmd_vel message
        cmd_vel.linear.x = 0.5; // Set linear velocity to 0.5 m/s
        cmd_vel.angular.z = 0.2; // Set angular velocity to 0.2 rad/s

        cmd_vel_publisher.publish(cmd_vel); // Publish the cmd_vel message
        ROS_INFO("cmd_vel linear.x = %lf, angular.z = %lf", cmd_vel.linear.x, cmd_vel.angular.z);

        ros::spinOnce();
        naptime.sleep();
    }
    return 0;
}
