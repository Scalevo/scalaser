#include "ros/ros.h"
#include "sensor_msgs/PointCloud.h"
#include "matlabCppInterface/Engine.hpp"
#include <boost/cstdint.hpp>
#include <math.h>

class matching{
  
  ros::NodeHandle nt;

  // Static Engine
  static matlab::Engine engine;  

  // Pointcloud Vector
  Eigen::VectorXf xi;		// X-Vector of pointcloud transformed around inital values
  Eigen::VectorXf zi;		// Z-Vector of pointcloud transformed around inital values
  Eigen::VectorXf xf;		// Common X-Vector of transformed pointcloud and optimized template
  Eigen::VectorXf zf;		// Z-Vector of transformed
  Eigen::VectorXf z_r;		// Z-Vector of optimized template

  // Transform Parameters
  float phi;
  float dzi;
  float fov_s;
  float fov_d;

  // Initial and Result Vector
  Eigen::VectorXf v0;
  Eigen::VectorXf v_r;

  // Error of fminsearch()
  float se_r;
  
  // Helper to transform each laser scan accordingly
  int h;


public:
  matching() {};
  matching(ros::NodeHandle n_,float phi_,float dzi_,float fov_s_,float fov_d_,std::vector<float> &v0_,int h_);
 // std::vector<float> getResultVector();
  void matchCallback(const sensor_msgs::PointCloud::ConstPtr& msg);
  void matchTemplate();
  float getDx() {return v_r(2);}
  
};

matlab::Engine matching::engine(true); // Initialize static matlab engine

matching::matching(ros::NodeHandle n_,float phi_,float dzi_,float fov_s_,float fov_d_,std::vector<float> &v0_,int h_):
 nt(n_),phi(phi_),dzi(dzi_),fov_s(fov_s_),fov_d(fov_d_),h(h_),xi(fov_d_),zi(fov_d_),xf(fov_d_),zf(fov_d_),z_r(fov_d_),v0(v0_.size()),v_r(v0_.size())
{
  if(engine.good()){ROS_INFO("Engine succesfully initialized.");}
  engine.changeWorkingDirectory("~/catkin_ws/src/scalaser/matlab"); 	// Path to matlab function
  for (int i=0;i<v0_.size();++i) v0(i) = v0_[i];			// Copy std::vector into Eigen::VectorXf
  v_r = v0;
}

void matching::matchCallback(const sensor_msgs::PointCloud::ConstPtr& msg)
{
  // Initialize Vector transformed around initial guess
  for (int i=0;i<fov_d;++i)
  {
   float a = msg->points[i+fov_s].x;
   float b = msg->points[i+fov_s].y;
   //xi.push_back(- b*sin(phi)*h + a*cos(phi));
   //zi.push_back(- a*cos(phi) - b*cos(phi)*h);

   //xi(i) = (- b*sin(phi)*h + a*cos(phi)); // Transform of Matlab
   //zi(i) = (- a*sin(phi) - b*cos(phi)*h);
   xi(i) = (- b*cos(phi)*h + a*sin(phi));
   zi(i) = (- a*cos(phi) - b*sin(phi)*h + dzi);
  }
}

void matching::matchTemplate()
{
  //ROS_INFO("Matching Template");
  engine.put("xi",xi);
  engine.put("zi",zi);

  engine.executeCommand("test(xi,zi)");

  engine.put("v0",v_r);
/*  engine.executeCommand("[v_r,se_r,z_r,xf,zf] = stairparam(xi,zi,v0);");
  engine.getEigen("v_r",v_r);
  engine.get("se_r",se_r);
  engine.get("z_r",z_r);			 
  engine.get("xf",xf);
  engine.get("zf",zf);
*/
}



class Angle{

private:
  ros::NodeHandle n;

  // Subscribers
  ros::Subscriber sub_1;
  ros::Subscriber sub_2;
  // Matching objects
  matching cloud_1;
  matching cloud_2;

  // Parameters of Base Transform
  int fov_s;
  int fov_d;
  float phi_1;
  float phi_2;
  float dzi;
  
  // Start Values of fminsearch() Vector
  std::vector<float> v0;
  float a;
  float beta;

  // Result of fminsearch
  float dx_1;
  float dx_2;

public:
  Angle(ros::NodeHandle n_);
  float computeAngle();
};


Angle::Angle(ros::NodeHandle n_):
n(n_),fov_s(311),fov_d(220),phi_1(-3*3.14/180),phi_2(1*3.14/180),dzi(.6),a(.63)
{
  // initialize start vector v0
  v0.push_back(.17);			// heigth - 		h0
  v0.push_back(.3);			// depth - 		t0
  v0.push_back(.05);			// phase offset - 	dx0
  v0.push_back(.0);			// sensor height - 	dz0
  v0.push_back(10*3.14/180);		// sensor rotation - 	phi0

  ROS_INFO("Start Values of fminsearch:");
  ROS_INFO("stair heigth________h0 = %f",v0[0]); 
  ROS_INFO("stair depth_________t0 = %f",v0[1]);
  ROS_INFO("phase offset_______dx0 = %f",v0[2]);
  ROS_INFO("sensor height______dz0 = %f",v0[3]);
  ROS_INFO("sensor rotation___phi0 = %f",v0[4]);
 
  // create & asssign temporary matching object
  matching cloud_1_t(n_,phi_1,dzi,fov_s,fov_d,v0,1);
  matching cloud_2_t(n_,phi_2,dzi,811-fov_s-fov_d,fov_d,v0,-1);

  cloud_1 = cloud_1_t;
  cloud_2 = cloud_2_t;

  // create subscriber
  sub_1 = n.subscribe("cloud_1",1, &matching::matchCallback, &cloud_1);
  sub_2 = n.subscribe("cloud_2",1, &matching::matchCallback, &cloud_2);
}

float Angle::computeAngle(){
  cloud_1.matchTemplate();
  dx_1 = cloud_1.getDx();
  cloud_2.matchTemplate();
  dx_2 = cloud_2.getDx();
  beta = atan((dx_1-dx_2)/a);
  return beta;
}



int main(int argc, char **argv) {

 ros::init(argc,argv,"angle");
 ros::NodeHandle n;
 Angle angle(n);
 ROS_INFO("Angle method constructed"); 

 float beta = 0;
 ros::Rate loop_rate(15); // [Hz]

  while (ros::ok())
  {
    beta = angle.computeAngle();
    ROS_INFO("Angle = %f",beta);
    loop_rate.sleep();
    ros::spinOnce();
  }

 return 0;
}
