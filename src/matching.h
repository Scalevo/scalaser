#ifndef MATCHING_H
#define MATCHING_H

#include <math.h>
#include <boost/cstdint.hpp>
#include <vector>
#include <string>

#include "ros/ros.h"
#include "sensor_msgs/PointCloud.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
#include "matlabCppInterface/Engine.hpp"
#include "matlabCppInterface/MatFile.hpp"




class matching{
  ros::NodeHandle n;

  // Cpp to MATLAB stuff
  static matlab::Engine engine;
  matlab::MatFile file;

  // Message Subscriber

  // Publishers
  ros::Publisher se_r_pub;

  // Messages
  sensor_msgs::PointCloud::ConstPtr temporary_msg;

  // Pointcloud Vector
  Eigen::VectorXd xi;        // X-Vector of pointcloud transformed around inital values
  Eigen::VectorXd zi;        // Z-Vector of pointcloud transformed around inital values
  Eigen::VectorXd xi_temp;   // X-Vector of pointcloud transformed around inital values
  Eigen::VectorXd zi_temp;   // Z-Vector of pointcloud transformed around inital values
  Eigen::VectorXd xf;        // Common X-Vector of transformed pointcloud and optimized template
  Eigen::VectorXd zf;        // Z-Vector of transformed
  Eigen::VectorXd z_r;       // Z-Vector of optimized template

  // Upper and lower Bounds of fmincon optimization

  Eigen::VectorXd lb;
  Eigen::VectorXd ub;

  // Initial and Result Vector
  Eigen::VectorXd v0;
  Eigen::VectorXd v_r;

  // Transform Parameters
  double phi0;
  double dzi;
  int fov_s;
  int fov_d;

  // Error of fminsearch()
  double se_r;

  // Helper to transform each laser scan accordingly
  int h;
  double threshold;

 public:
  matching() {}
  matching(ros::NodeHandle n_, double phi0_, double dzi_, int fov_s_, int fov_d_, std::vector<double> &v0_, int h_);
  // std::vector<double> getResultVector();
  void matchCallback(const sensor_msgs::PointCloud::ConstPtr& msg);
  void transformMsg(const sensor_msgs::PointCloud::ConstPtr& msg);
  void matchTemplate();
  void updateParameters(double phi0_, double dzi_, int fov_s_, int fov_d_);
  void publishSe_r();
  void setData() {xi = xi_temp; zi = zi_temp;}
  void fillMatfile();
  void fillEngine();  // Slow way of sending data to matlab

  // Return functions
  Eigen::VectorXd getV_r() {return v_r;}
  double getDx() {return fmod(v_r(2), sqrt(v_r(0)*v_r(0)+v_r(1)*v_r(1)));}
  double getSe_r() {return se_r;}
};

#endif

