#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sittwe_driver/rpm.h>
#include "robot_specs.h"

int desire_right = 0;
int desire_left  = 0;

float L = track_width / 2.0;
float ms_to_rpm_ratio = 60.0/ (pi * wheel_diameter);

void twistCallback( const geometry_msgs::Twist& cmd_msg) 
{
	double x = cmd_msg.linear.x;
	double theta = cmd_msg.angular.z;
	double right = 1.0 * x + (theta * L);
	double left  = 1.0 * x - (theta * L);
	right = right * ms_to_rpm_ratio;
	left  = left  * ms_to_rpm_ratio;

	desire_right = right;
	desire_left  = left; 
}

int main(int argc,char** argv)
{
	ros::init(argc,argv,"twist_to_motors");
	ros::NodeHandle nh;
	ros::Subscriber sub = nh.subscribe("cmd_vel", 50, twistCallback);
	ros::Publisher  pub = nh.advertise<sittwe_driver::rpm>("desire_rpm", 50);

	sittwe_driver::rpm RPM;
	ros::Rate r(10);

	while(ros::ok())
	{
		RPM.desire_rpm_right = desire_right;
		RPM.desire_rpm_left  = desire_left;

		pub.publish(RPM);

		ros::spinOnce();
		r.sleep();

	}
	return 0;
}

