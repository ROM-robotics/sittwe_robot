#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
float pi_ = 3.141592; float two_pi = 6.283184;
double rpm_1 = 0.041253236 * 1.0;
double rpm_10 = 0.041253236 * 10.0; double rpm_15 = 0.041253236 * 15.0;

float normalize_angle(float angle)
{
    float res = angle;
    while(res > pi_)
    {
        res -= two_pi;
    }
    while(res < -pi_)
    {
        res += two_pi;
    }
    return res;
}

float degree_to_radian(float angle)
{
    return (angle*(pi_/180.0));
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rotate");
    ros::NodeHandle n;
    ros::NodeHandle nh_private_("~");
    ros::Publisher pub=n.advertise<geometry_msgs::Twist>("/cmd_vel",50);
    tf::TransformListener listener;
    tf::StampedTransform transform;
    int rate = 10;
    ros::Rate r(rate);

    double angular_velocity = rpm_15;  

    geometry_msgs::Twist move_cmd;
    move_cmd.linear.x = 0.0;


    float degree = 0.0;
    nh_private_.getParam("rotate_degree", degree);


    if( degree > 0 )       { move_cmd.angular.z = angular_velocity;    }
    else if ( degree < 0)  { move_cmd.angular.z = -angular_velocity;   }

    ros::Duration(1).sleep();
    
    float goal_angel = degree_to_radian(degree);               
    float angular_tolarance=0.0175;                         // 1 degree

    float time = goal_angel/angular_velocity;
    int time_ = ceil(time);
    int ticks = rate * time_;

    move_cmd.linear.x = 0;
    for(int i=0;i<ticks;i++){
        pub.publish(move_cmd);
        r.sleep();
    }

    // stop
    for( int i=0;i<10;i++)
    {
        move_cmd.linear.x = 0.00000;
        move_cmd.angular.z= 0.00000;
        pub.publish(move_cmd);
        r.sleep();
    }
    ros::Duration(1).sleep();   
    ROS_INFO("NODE will shutdown!");
    ros::shutdown();
    return 0;
}

