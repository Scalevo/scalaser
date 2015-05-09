#ifndef ANGLE_H
#define ANGLE_H

#include "ros/ros.h"
#include <boost/cstdint.hpp>
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
#include <tf/transform_broadcaster.h>
#include <math.h>
#include "matching.h"
#include "message_filters/subscriber.h"
#include "message_filters/cache.h"




class Angle{

private:
  ros::NodeHandle n;

  // Subscribers
  ros::Subscriber sub_1;
  ros::Subscriber sub_2;

  // Publishers
  ros::Publisher pub_1;		     // beta
  ros::Publisher pub_2;		     // stairParam
  ros::Publisher pub_velocity;

  // Tf
  tf::TransformBroadcaster br;
  tf::Transform transform;

  // Timers
  ros::Timer main_timer;

  // Matching objects
  matching cloud_1;
  matching cloud_2;

  // Parameters of Base Transform
  int fov_s;
  int fov_d;
  double phi0;
  double dzi;
  
  // Start Values of fminsearch() Vector
  std::vector<double> v0;
  double a;


  // Average of results of fminsearch
  Eigen::VectorXd v_s;

  // Messages
  std_msgs::Float64MultiArray stair_param;
  std_msgs::Float64 beta;
  std_msgs::Float64MultiArray velocity;

  // Beta
  double beta_new;
  double beta_old;

  // Parameters for motor controler
  double kp;
  double threshold;

public:
  Angle(ros::NodeHandle n_);

  void timerCallback(const ros::TimerEvent& event);

  void computeAngle();
  void computeStair();
  void computeVelocity();

  void setPosition();
  void setParameters();
};

#endif