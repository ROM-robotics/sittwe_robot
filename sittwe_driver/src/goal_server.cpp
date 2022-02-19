#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <actionlib/server/simple_action_server.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/PoseStamped.h>

#include <geometry_msgs/TransformStamped.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

class GoalServer
{
public:
    
  GoalServer(std::string name) : 
    as_(nh_, name, false),
    action_name_(name)
  {
    //register the goal and feeback callbacks
    as_.registerGoalCallback(boost::bind(&GoalServer::goalCB, this));
    as_.registerPreemptCallback(boost::bind(&GoalServer::preemptCB, this));

    //subscribe to the data topic of interest
    sub_ = nh_.subscribe("/emergency_stop", 1, &GoalServer::emergencyCB, this);
    as_.start();
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

  void goalCB()
  {
    target_pose = as_.acceptNewGoal()->target_pose;
    double target_x = target_pose.pose.position.x;   double target_y = target_pose.pose.position.y;
    ROS_INFO_STREAM("X=" << target_x << "Y=" << target_y << "W=" << target_pose.pose.orientation.w );
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

  void preemptCB()
  {
    ROS_INFO("%s: Preempted", action_name_.c_str());
    // set the action state to preempted
    as_.setPreempted();
  }

  void emergencyCB(const std_msgs::Float32::ConstPtr& msg)
  {/*
    // make sure that the action hasn't been canceled
    if (!as_.isActive())
      return;
    
    data_count_++;
    feedback_.sample = data_count_;
    feedback_.data = msg->data;
    //compute the std_dev and mean of the data 
    sum_ += msg->data;
    feedback_.mean = sum_ / data_count_;
    sum_sq_ += pow(msg->data, 2);
    feedback_.std_dev = sqrt(fabs((sum_sq_/data_count_) - pow(feedback_.mean, 2)));
    as_.publishFeedback(feedback_);

    if(data_count_ > goal_) 
    {
      result_.mean = feedback_.mean;
      result_.std_dev = feedback_.std_dev;

      if(result_.mean < 5.0)
      {
        ROS_INFO("%s: Aborted", action_name_.c_str());
        //set the action state to aborted
        as_.setAborted(result_);
      }
      else 
      {
        ROS_INFO("%s: Succeeded", action_name_.c_str());
        // set the action state to succeeded
        as_.setSucceeded(result_);
      }
    } 
    */
  }

protected:
    
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<move_base_msgs::MoveBaseAction> as_;
  std::string action_name_;
  geometry_msgs::PoseStamped target_pose;
  move_base_msgs::MoveBaseActionFeedback feedback_;
  move_base_msgs::MoveBaseActionResult result_;
  ros::Subscriber sub_;
  tf::TransformListener listener;
  int rate=10;
  ros::Rate r=10;
  float constant_lin_vel = 0.00408 * 20.0;           // 20 rpm
  float angular_velocity = 0.4120;  
  geometry_msgs::Twist move_cmd;
  ros::Publisher pub=nh_.advertise<geometry_msgs::Twist>("/cmd_vel",50);
  float pi_ = 3.141592; float two_pi = 6.283184;
  float deg_to_rad_constant = 3.141592/180.0;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_base");

  GoalServer goal_server(ros::this_node::getName());
  ros::spin();

  return 0;
}