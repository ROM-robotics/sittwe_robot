#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
int main(int argc, char** argv)
{
    ros::init(argc, argv, "stop");
    ros::NodeHandle n;
    ros::Publisher pub=n.advertise<geometry_msgs::Twist>("/cmd_vel",50);
    int rate = 10;
    
    geometry_msgs::Twist move_cmd;
    move_cmd.linear.x = 0.0;                              
    move_cmd.angular.z = 0.0;

    ros::Rate r(rate);

    for( int i=0;i<100;i++)
    {
        pub.publish(move_cmd);
        r.sleep();
    }
    
    ros::Duration(1).sleep();   

    ros::shutdown();
    return 0;
}

