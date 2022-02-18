#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib_tutorials/FibonacciAction.h>
#include <move_base_msgs/MoveBaseAction.h>

class GoalServer
{
protected:

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<move_base_msgs::MoveBaseAction> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
  std::string action_name_;
  // create messages that are used to published feedback/result
  move_base_msgs::MoveBaseActionFeedback feedback_;
  move_base_msgs::MoveBaseActionResult result_;

public:

  GoalServer(std::string name) :
    as_(nh_, name, boost::bind(&GoalServer::executeCB, this, _1), false),
    action_name_(name)
  {
    as_.start();
  }

  ~GoalServer(void)
  {
  }

  void executeCB(const move_base_msgs::MoveBaseActionGoalConstPtr &goal)
  {
    // helper variables
    ros::Rate r(10);
    bool success = true;
    ROS_INFO("receive goal");

    // start executing the action
    while(true)
    {
      // check that preempt has not been requested by the client
      if (as_.isPreemptRequested() || !ros::ok())
      {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        as_.setPreempted();
        success = false;
        break;
      }
      // publish the feedback
      //feedback_.status = 1;
      //as_.publishFeedback(feedback_);
      // this sleep is not necessary, the sequence is computed at 1 Hz for demonstration purposes
      r.sleep();
    }

    if(success)
    {
      //result_.sequence = feedback_.sequence;
      ROS_INFO("%s: Succeeded", action_name_.c_str());
      // set the action state to succeeded
      as_.setSucceeded(result_);
    }
  }


};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "goal_server");

  GoalServer goal_server("move_base");
  ros::spin();

  return 0;
}