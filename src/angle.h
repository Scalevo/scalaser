#ifndef ANGLE_H
#define ANGLE_H

#include "ros/ros.h"
#include <boost/cstdint.hpp>
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
#include <tf/transform_broadcaster.h>
#include <math.h>
#include "matching.h"

class Angle{

private:
  ros::NodeHandle n;

  // Subscribers
  ros::Subscriber sub_1;
  ros::Subscriber sub_2;
  // Publishers
  ros::Publisher pub_1;		// beta
  ros::Publisher pub_2;		// stairParam

  // Tf
  tf::TransformBroadcaster br;
  tf::Transform transform;

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


  // Result of fminsearch
  Eigen::VectorXd v_s;

  std_msgs::Float64MultiArray stair_param;
  std_msgs::Float64 beta;

public:
  Angle(ros::NodeHandle n_);
  void computeAngle();
  void computeStair();
  void SetPosition();
};

#endif
