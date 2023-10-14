//driver
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
