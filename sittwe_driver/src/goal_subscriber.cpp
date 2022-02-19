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

  void goX(double x)
  {
    float distance = 0;
    while( distance > abs(x) )
    {
      move_cmd.linear.x = constant_lin_vel;
      move_cmd.angular.z= 0.0;
      pub.publish(move_cmd);
      r.sleep();

      tf::StampedTransform currentTF = checkTF();
      double current_x = currentTF.getOrigin().x();
      distance = abs(x - current_x);
    }
  }

  void goY(double y)
  {
    float distance = 0;
    while( distance > abs(y) )
    {
      move_cmd.linear.x = constant_lin_vel;
      move_cmd.angular.z= 0.0;
      pub.publish(move_cmd);
      r.sleep();

      tf::StampedTransform currentTF = checkTF();
      double current_y = currentTF.getOrigin().y();
      distance = abs(y - current_y);
    }
  }

  void rot(int degree)
  {
    double goal_angel = abs(degree)*deg_to_rad_constant;
    
    tf::StampedTransform currentTF = checkTF();
    
    tf::Matrix3x3 m( currentTF.getRotation() );
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    double last_angle = yaw;
    double turn_angle = 0;

    if( degree > 0 )       { move_cmd.angular.z = angular_velocity;    }
    else if ( degree < 0)  { move_cmd.angular.z = -angular_velocity;   }

    while( abs(turn_angle) < abs(goal_angel) )
    {
        ROS_INFO("moving ..");
        pub.publish(move_cmd);
        r.sleep();

        tf::StampedTransform currentTF = checkTF();

        tf::Matrix3x3 m( currentTF.getRotation() );
        m.getRPY(roll,pitch,yaw);
        double rotation = yaw;
        //ROS_INFO_STREAM("current Yaw"<< yaw);
        double delta_angle = normalize_angle(rotation- last_angle);
        turn_angle += delta_angle;
        last_angle = rotation;
    }

  }

  void goalCB(const geometry_msgs::PoseStamped::ConstPtr& target_pose)
  {
    
    double target_x = target_pose->pose.position.x;   double target_y = target_pose->pose.position.y;
    ROS_INFO_STREAM("X=" << target_x << "Y=" << target_y << "W=" << target_pose->pose.orientation.w );
    int quardrant = checkQuardrant(target_x, target_y);

    tf::StampedTransform currentTF = checkTF();

    double current_x = currentTF.getOrigin().x();
    double current_y = currentTF.getOrigin().y();

    double diff_x = target_x - current_x;
    double diff_y = target_y - current_y;
    
    switch (quardrant)
    {
    case 1:
      goY(diff_y); rot(90); goX(diff_x); rot(-90);
      break;
    case 2:
      goY(diff_y); rot(-90); goX(diff_x); rot(90);
      break;
    case 3:
      rot(180); goY(diff_y); rot(-90); goX(diff_x); rot(-90);
      break;
    case 4:
      rot(-180); goY(diff_y); rot(90); goX(diff_x); rot(90);
      break;
    default:
      ROS_INFO("Switch(quardrant) Error!");
      break;
    }

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
  int rate=10;
  ros::Rate r=10;
  float constant_lin_vel = 0.00408 * 20.0;           // 20 rpm
  float angular_velocity = 0.4120;  
  geometry_msgs::Twist move_cmd;
  ros::Publisher pub=nh_.advertise<geometry_msgs::Twist>("/cmd_vel",50);
  float pi_ = 3.141592; float two_pi = 6.283184;
  float deg_to_rad_constant = 3.141592/180.0;
  bool emergency_stop = false;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_base_subscriber");

  GoalServer goal_server(ros::this_node::getName());
  ros::spin();

  return 0;
}