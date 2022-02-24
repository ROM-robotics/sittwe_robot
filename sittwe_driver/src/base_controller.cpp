#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <sittwe_driver/actual_rpm.h>
#include <stdio.h>
#include <cmath>
#include <algorithm>
#include "robot_specs.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

double right_act_rpm = 0.0;
double left_act_rpm = 0.0;

double delta_time = 0.0;
double x_pos = 0.0;
double y_pos = 0.0;
double theta = 0.0;
float imu_yaw = 0.0;
bool use_imu = false;

double delta_x = 0.0;
double delta_y = 0.0;
double delta_theta = 0.0;

ros::Time current_time;
ros::Time rpm_time(0.0);

void handle_rpm( const sittwe_driver::actual_rpm& rpm) 
{
  right_act_rpm = rpm.actual_right;
  left_act_rpm  = rpm.actual_left;
  delta_time    = rpm.delta_time;
  imu_yaw = rpm.imu_yaw;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "base_controller");
  ros::NodeHandle n;
  ros::NodeHandle nh_private_("~");
  ros::Subscriber sub = n.subscribe("actual_rpm", 50, handle_rpm);
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
  tf::TransformBroadcaster broadcaster;

  double rate = 10.0;
  double linear_scale_positive = 1.0;
  double linear_scale_negative = 1.0;
  double angular_scale_positive = 1.0;
  double angular_scale_negative = 1.0;
  double angular_scale_accel = 1.0;

  double alpha = 0.0;

  double dt = 0.0;
  double x = 0.0;
  double y = 0.0;

  double angular_displacement = 0.0;
  double dth = 0.0;

  double linear_displacement = 0.0;
  double vx = 0.0;
  double vy = 0.0;
  double vth = 0.0;

  char base_link[] = "base_link";
  char odom[] = "/odom";
  ros::Duration d(1.0);

  nh_private_.getParam("publish_rate", rate);
  nh_private_.getParam("linear_scale_positive", linear_scale_positive);
  nh_private_.getParam("linear_scale_negative", linear_scale_negative);
  nh_private_.getParam("angular_scale_positive", angular_scale_positive);
  nh_private_.getParam("angular_scale_negative", angular_scale_negative);
  nh_private_.getParam("angular_scale_accel", angular_scale_accel);
  nh_private_.getParam("use_imu", use_imu);
  nh_private_.getParam("alpha", alpha);

  ros::Rate r(rate);

  while(n.ok()){
    ros::spinOnce();
    current_time = ros::Time::now();
    dt = delta_time;    
    
    float linear_velocity = (right_act_rpm+left_act_rpm)*0.001806416;//wheel_diameter*pi/(60*2);
    linear_displacement   = linear_velocity * dt;

    float angular_velocity= (right_act_rpm-left_act_rpm)*0.003612831/track_width;//wheel_diameter*pi/(60*track_width);
    angular_displacement  = angular_velocity*dt;

    dth = alpha*angular_displacement;

    if (dth > 0) dth *= angular_scale_positive;
    if (dth < 0) dth *= angular_scale_negative;
    if (linear_displacement > 0) linear_displacement *= linear_scale_positive;
    if (linear_displacement < 0) linear_displacement *= linear_scale_negative;
//-----------------------------------------------------------------//
    x = cos(dth) * linear_displacement;
    y = -sin(dth) * linear_displacement; 

    delta_x = (cos(theta) * x - sin(theta) * y);
    delta_y = (sin(theta) * x + cos(theta) * y);
    delta_theta = dth;

    x_pos += delta_x;
    y_pos += delta_y;
    theta += delta_theta;

    if(theta >= two_pi) theta -= two_pi;
    if(theta <= -two_pi) theta += two_pi;

    geometry_msgs::Quaternion odom_quat;
    if(use_imu) 
    {
      odom_quat = tf::createQuaternionMsgFromYaw(imu_yaw);
    }
    else {
      odom_quat = tf::createQuaternionMsgFromYaw(theta);
    }
    
    
      geometry_msgs::TransformStamped t;
      t.header.frame_id = odom;
      t.child_frame_id = base_link;
      t.transform.translation.x = x_pos;
      t.transform.translation.y = y_pos;
      t.transform.translation.z = 0.0;
      t.transform.rotation = odom_quat;
      t.header.stamp = current_time;

      broadcaster.sendTransform(t);
    

    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = current_time;
    odom_msg.header.frame_id = odom;
    odom_msg.pose.pose.position.x = x_pos;
    odom_msg.pose.pose.position.y = y_pos;
    odom_msg.pose.pose.position.z = 0.0;
    odom_msg.pose.pose.orientation = odom_quat;


    if (right_act_rpm == 0 && left_act_rpm == 0){
      odom_msg.pose.covariance[0] = 1e-9;
      odom_msg.pose.covariance[7] = 1e-3;
      odom_msg.pose.covariance[8] = 1e-9;
      odom_msg.pose.covariance[14] = 1e6;
      odom_msg.pose.covariance[21] = 1e6;
      odom_msg.pose.covariance[28] = 1e6;
      odom_msg.pose.covariance[35] = 1e-9;
      odom_msg.twist.covariance[0] = 1e-9;
      odom_msg.twist.covariance[7] = 1e-3;
      odom_msg.twist.covariance[8] = 1e-9;
      odom_msg.twist.covariance[14] = 1e6;
      odom_msg.twist.covariance[21] = 1e6;
      odom_msg.twist.covariance[28] = 1e6;
      odom_msg.twist.covariance[35] = 1e-9;
    }
    else{
      odom_msg.pose.covariance[0] = 1e-3;
      odom_msg.pose.covariance[7] = 1e-3;
      odom_msg.pose.covariance[8] = 0.0;
      odom_msg.pose.covariance[14] = 1e6;
      odom_msg.pose.covariance[21] = 1e6;
      odom_msg.pose.covariance[28] = 1e6;
      odom_msg.pose.covariance[35] = 1e3;
      odom_msg.twist.covariance[0] = 1e-3;
      odom_msg.twist.covariance[7] = 1e-3;
      odom_msg.twist.covariance[8] = 0.0;
      odom_msg.twist.covariance[14] = 1e6;
      odom_msg.twist.covariance[21] = 1e6;
      odom_msg.twist.covariance[28] = 1e6;
      odom_msg.twist.covariance[35] = 1e3;
    }
    //vx = (dt == 0)?  0 : linear_displacement/dt;
    //vth = (dt == 0)? 0 : dth/dt;
    odom_msg.child_frame_id = base_link;
    odom_msg.twist.twist.linear.x = linear_velocity;
    odom_msg.twist.twist.linear.y = 0.0;
    odom_msg.twist.twist.angular.z = angular_velocity;

    odom_pub.publish(odom_msg);
    r.sleep();
  }
}
