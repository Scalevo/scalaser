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

#define PI 3.14159265


class matching{
  ros::NodeHandle n;

  // Cpp to MATLAB
  static matlab::Engine engine;
  matlab::MatFile file;

  // Publishers
  ros::Publisher se_r_pub;

  // Pointcloud Vector
  Eigen::VectorXd xi;        // X-Vector of pointcloud transformed around inital values
  Eigen::VectorXd zi;        // Z-Vector of pointcloud transformed around inital values
  Eigen::VectorXd xf;        // Common X-Vector of transformed pointcloud and optimized template
  Eigen::VectorXd zf;        // Z-Vector of transformed
  Eigen::VectorXd z_r;       // Z-Vector of optimized template

  std::vector<double> xi_temp;
  std::vector<double> zi_temp;
  std::vector<double> xi_match;
  std::vector<double> zi_match;

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
  double ang_inc;
  double min_ang;
  double max_ang;

  // Error of fminsearch()
  double se_r;

  // Helper to transform each laser scan accordingly
  int h;
  double threshold;


 public:
  // Constructors
  matching() {}
  matching(ros::NodeHandle n_, double phi0_, double dzi_, int fov_s_, int fov_d_, Eigen::VectorXd v0_, int h_);

  // main functions
  void matchCallback(const sensor_msgs::PointCloud::ConstPtr& msg);
  void transformMsg();
  void matchTemplate();

  // cpp to matlab connection functions
  void fillMatfile(); // Fast way of sending data to matlab
  void fillEngine();  // Slow way of sending data to matlab

  // Parameter update functions
  void setParameters(double phi0_, double dzi_, int fov_s_, int fov_d_);
  void setData() {
    xi_match = xi_temp;
    zi_match = zi_temp;
    // xi = xi_temp; zi = zi_temp;
  }
  void setFminArgs(Eigen::VectorXd v_r_); 

  // Return functions
  Eigen::VectorXd getV_r() {return v_r;}
  // double getDx() {return fmod(v_r(2), sqrt(v_r(0)*v_r(0)+v_r(1)*v_r(1)));}
  double getDx() {return v_r(2);}
  double getSe_r() {return se_r;}
  double getDiag() {return sqrt(v_r(1)*v_r(1) + v_r(2)*v_r(2));}

  // Publishers
  void publishSe_r();

};

#endif

