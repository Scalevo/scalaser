#ifndef ANGLE_H
#define ANGLE_H

#include <boost/cstdint.hpp>
#include <math.h>
#include <fstream>

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include "tf/transform_broadcaster.h"
#include "matlabCppInterface/Engine.hpp"
#include "matlabCppInterface/MatFile.hpp"

#include "scalevo_msgs/Starter.h"


#include "matching.h"

#define PI 3.14159265


class Angle{

private:
  ros::NodeHandle n;

  matlab::Engine plot_engine;
  // Status Server
  ros::ServiceServer service;

  // Subscribers
  ros::Subscriber sub_1;
  ros::Subscriber sub_2;

  ros::Subscriber sub_joint;

  // Publishers
  ros::Publisher pub_1;		     // beta
  ros::Publisher pub_2;		     // stairParam
  // ros::Publisher pub_velocity;
  ros::Publisher pub_s_velocity;



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

  // Wheelchair Parameters
  double r_h;
  double s;
  double phi_f;
  
  // Start Values of fminsearch() Vector
  Eigen::VectorXd v0;
  double a;

  // Average of results of fminsearch
  Eigen::VectorXd v_s;

  // Vectors for Result evaluation
  double time_start;
  std::vector<double> beta_vector;
  std::vector<double> alpha_vector;
  std::vector<double> time_vector;
  std::vector<double> dx_1_vector;
  std::vector<double> dx_2_vector;

  // Messages
  std_msgs::Float64MultiArray stair_param;
  std_msgs::Float64 beta;
  std_msgs::Float64MultiArray velocity;

  // Beta
  double beta_new;
  double beta_old;

  // Alpha
  double alpha;
  double alpha_1;
  double alpha_2;
  double dx_1;
  double dx_2;
  double diag_1;
  double diag_2;

  // Parameters for motor controler
  double kp;
  double vel_fwd;
  double threshold;

  // counter
  int count;
  int wrong_beta_count;

public:
  Angle(ros::NodeHandle n_);

  void initializeMatching();
  void timerCallback(const ros::TimerEvent& event);
  void jointCallback(const sensor_msgs::JointState::ConstPtr& joint_state);

  bool alignWheelchair(scalevo_msgs::Starter::Request& request, scalevo_msgs::Starter::Response& response);

  void computeAngle();
  void computeAlpha();
  void computeBeta();
  void computeStair();
  void computeVelocity();

  void setBoundaries();
  void setPosition();
  void setParameters();

  void plot_data(std::vector<double> data_vector);
};

#endif