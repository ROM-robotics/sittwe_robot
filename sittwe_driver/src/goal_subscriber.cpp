#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/PoseStamped.h>

#include <geometry_msgs/TransformStamped.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

class GoalServer
{
public:
    
  GoalServer(std::string name) : 
    node_name_(name)
  {
    //subscribe to the data topic of interest
    sub_ = nh_.subscribe("/emergency_stop", 1, &GoalServer::emergencyCB, this);
    goal_sub = nh_.subscribe("/move_base_simple/goal", 1, &GoalServer::goalCB, this);

    rate=10;
    constant_lin_vel = 0.00408 * 20.0;           // 20 rpm 20rpm => 0.08168140
    angular_velocity = 0.041253236 * 10.0;    // 10 rpm 
    pi_ = 3.141592; float two_pi = 6.283184;
    deg_to_rad_constant = 3.141592/180.0;
    emergency_stop = false;
  }

  ~GoalServer(void) { }
  
  int checkQuardrant(double x, double y)
  {
    if(x > 0 && y > 0 ) {return 1;} // Quardrant I
    else if(x < 0 && y > 0 ) {return 2;} // Quardrant II
    else if(x < 0 && y < 0 ) {return 3;} // Quardrant III
    else if(x > 0 && y < 0 ) {return 4;} // Quardrant IV
  }

  tf::StampedTransform checkTF()
  {
    tf::StampedTransform transform;
        try     {   listener.waitForTransform("odom","base_link", ros::Time(0), ros::Duration(1.0));            }
        catch(tf::LookupException e)    
                {   ROS_INFO_STREAM("Cannot wait tf between /odom and /baselink. Error= "<< e.what()<<"\n");    }

        try     {   listener.lookupTransform("odom","base_link", ros::Time(0), transform);                      }
        catch(tf::TransformException e)
                {   ROS_INFO_STREAM("Cannot get tf between /odom and /baselink. Error= "<< e.what()<<"\n");     } 
    //ROS_INFO_STREAM("Acquire.. transform");
    return transform;
  }

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

  void goX(double x, double ref_x, double ref_y)
  {
    double target_dist = x;
    double distance = 0;
    ROS_INFO(" goX(): ");
    ROS_INFO_STREAM(" distance = "<< target_dist); ROS_INFO_STREAM(" linear_velocity = "<< constant_lin_vel); 
    while( distance < target_dist ) 
    {
      ROS_INFO(" goX() =>  while");
      move_cmd.linear.x = constant_lin_vel;
      move_cmd.angular.z= 0.0;
      pub.publish(move_cmd);
      r.sleep();

      tf::StampedTransform currentTF = checkTF();
      double current_distance = sqrt(  pow(ref_x - currentTF.getOrigin().x(), 2) + pow(ref_y - currentTF.getOrigin().y(), 2)  );
      distance = std::abs(current_distance);
    }
    ROS_INFO(" goX() # END");
  }

  void rot(double radian)
  {
    double goal_angel = std::abs(radian);
    // ROS_INFO_STREAM("radian = " << radian );
    // ROS_INFO_STREAM("abs(radian) = " << abs(radian) ); // error
    ROS_INFO_STREAM("std::abs(radian) = " << std::abs(radian) );
    tf::StampedTransform currentTF = checkTF();
    
    tf::Matrix3x3 m( currentTF.getRotation() );
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    double last_angle = yaw;
    double turn_angle = 0;

    if( radian >= 0 )       { move_cmd.angular.z = angular_velocity;    }
    else if ( radian < 0)  { move_cmd.angular.z = -angular_velocity;   }
    move_cmd.linear.x = 0.0;
    ROS_INFO(" rot():"); ROS_INFO_STREAM("ANGULAR VELOCITY" << move_cmd.angular.z );
    ROS_INFO_STREAM("turn_angle = "<< turn_angle << " , goal_angel = " << goal_angel);
    while( std::abs(turn_angle) < std::abs(goal_angel) )
    {
        ROS_INFO("rot() => while loop");
        pub.publish(move_cmd);
        r.sleep();

        tf::StampedTransform currentTF = checkTF();

        tf::Matrix3x3 m( currentTF.getRotation() );
        m.getRPY(roll,pitch,yaw);
        double rotation = yaw;
        ROS_INFO_STREAM("rot() => current Yaw"<< yaw);
        double delta_angle = normalize_angle(rotation- last_angle);
        turn_angle += delta_angle;
        last_angle = rotation;
    }
    ROS_INFO(" rot() # END");
  }

  void stop()
  {
    move_cmd.linear.x = 0.0;
    move_cmd.angular.z= 0.0;
    for(int i=0;i<10;i++)
    {
      pub.publish(move_cmd);
      r.sleep();
    }
  }

  void goalCB(const geometry_msgs::PoseStamped::ConstPtr& target_pose)
  {
    
    double target_x = target_pose->pose.position.x;   double target_y = target_pose->pose.position.y;
    ROS_INFO_STREAM("X = " << target_x << ", Y = " << target_y << ", W = " << target_pose->pose.orientation.w );
    int quardrant = checkQuardrant(target_x, target_y);
    ROS_INFO_STREAM("Quardrant = "<< quardrant);

    tf::StampedTransform currentTF = checkTF();
    double r_distance, theta; 

    r_distance = sqrt(  pow(target_x - currentTF.getOrigin().x(), 2) + pow(target_y - currentTF.getOrigin().y(), 2)  );
    theta = atan2( target_y - currentTF.getOrigin().y() , target_x - currentTF.getOrigin().x() ); 
    
    /*  
      atan = gives angle value between -90 and 90
      atan2 = gives angle value between -180 and 180
    */
    ROS_INFO_STREAM("r_distance = " << r_distance << ", theta = "<< theta);

    rot(theta);
    goX(r_distance, currentTF.getOrigin().x(), currentTF.getOrigin().y());
    rot(-theta); // +1 degree
    stop();
    
  }

  void emergencyCB(const std_msgs::Int8::ConstPtr& msg)
  {
    int flag = msg->data;
    if(flag == 1) { emergency_stop = true; }
    else { emergency_stop = false; };
  }

protected:
    
  ros::NodeHandle nh_;
  std::string node_name_;
  geometry_msgs::PoseStamped target_pose;
  ros::Subscriber sub_;
  ros::Subscriber goal_sub;
  tf::TransformListener listener;
  int rate;
  ros::Rate r=10;
  float constant_lin_vel;           // 20 rpm
  float angular_velocity; 
  geometry_msgs::Twist move_cmd;
  ros::Publisher pub=nh_.advertise<geometry_msgs::Twist>("/cmd_vel",50);
  float pi_,two_pi;
  float deg_to_rad_constant;
  bool emergency_stop;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_base_subscriber");

  GoalServer goal_server(ros::this_node::getName());
  ros::spin();

  return 0;
}