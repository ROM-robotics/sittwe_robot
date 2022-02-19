#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

float yourRPM = 20.0;

float OneRPM_per_linX = 0.00408;                           // pololu can run 1 rpm.
float min_lin_velocity = OneRPM_per_linX * 5.0;            //  5 rpm
float max_lin_velocity = OneRPM_per_linX * 30.0;           // 30 rpm

float constant_lin_vel = OneRPM_per_linX * yourRPM;           // 20 rpm

int main(int argc, char** argv)
{
    ros::init(argc, argv, "forward");
    ros::NodeHandle n;
    ros::NodeHandle nh_private_("~");
    ros::Publisher pub=n.advertise<geometry_msgs::Twist>("/cmd_vel",50);
    tf::TransformListener listener;
    tf::StampedTransform transform;
    int rate = 10;
    
    geometry_msgs::Twist move_cmd;
    move_cmd.linear.x = 0.0;                                   // for smooth
    
    ros::Rate r(rate);

    ros::Duration(1).sleep();
    // move_cmd.linear.x += min_lin_velocity;

    double linear_scale = 0.0;
    n.getParam("/linear_scale", linear_scale);
    float dis = 0.0;
    nh_private_.getParam("forward_meter", dis);

    float goal_distance = dis * linear_scale;    // meter

        try     {   listener.waitForTransform("odom","base_link", ros::Time(0), ros::Duration(1.0));            }
        catch(tf::LookupException e)    
                {   ROS_INFO_STREAM("Cannot wait tf between /odom and /baselink. Error= "<< e.what()<<"\n");    }

        try     {   listener.lookupTransform("odom","base_link", ros::Time(0), transform);                      }
        catch(tf::TransformException e)
                {   ROS_INFO_STREAM("Cannot get tf between /odom and /baselink. Error= "<< e.what()<<"\n");     } 

    float x_start = transform.getOrigin().x();
    float y_start = transform.getOrigin().y();
    float distance = 0;
    
    move_cmd.linear.x = constant_lin_vel;

    while(distance < goal_distance)
    { 
        pub.publish(move_cmd);
        r.sleep();

        try
        {
            listener.lookupTransform("odom","base_link", ros::Time(0), transform);
        }
        catch(tf::TransformException e)
        {
            ROS_INFO_STREAM("Cannot get tf between /odom and /baselink. Error= "<< e.what()<<"\n");
        } 

        distance = sqrt(pow(transform.getOrigin().x() - x_start, 2) + pow(transform.getOrigin().y() - y_start, 2) );

     
    } //-----------------------------------------------------------------------------------------------------------------
        
   
    // stop
    for( int i=0;i<10;i++)
    {
        move_cmd.linear.x = 0.0;
        pub.publish(move_cmd);
        r.sleep();
    }
    
    ros::Duration(1).sleep();   

    ros::shutdown();
    return 0;
}

