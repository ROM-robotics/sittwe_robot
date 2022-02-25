#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

double lin_x = 0.0; double ang_z = 0.0;

void handleJOY( const geometry_msgs::Twist& twist2) 
{
  lin_x = twist2.linear.x / 4.5;
  ang_z = twist2.angular.z/ 4.5;
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "manual_and");
  ros::NodeHandle n;
  ros::Subscriber sub2 = n.subscribe("/joy_teleop/cmd_vel", 50, handleJOY);
  ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 50);
  ros::Rate r(10);

  geometry_msgs::Twist move_cmd;
  move_cmd.linear.x = 0.0;
  move_cmd.linear.y = 0.0;
  move_cmd.linear.z = 0.0;
  move_cmd.angular.x= 0.0;
  move_cmd.angular.y= 0.0;
  move_cmd.angular.z= 0.0;

  while(n.ok()){
    ros::spinOnce();
    move_cmd.linear.x = lin_x;
    move_cmd.angular.z= ang_z;

    pub.publish(move_cmd);
    r.sleep();
  }
}
