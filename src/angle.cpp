#include "ros/ros.h"
#include "sensor_msgs/PointCloud.h"
#include "laser_geometry/laser_geometry.h"
#include "geometry_msgs/Point32.h"
#include "std_msgs/Float32.h"
#include "matlabCppInterface/Engine.hpp"
#include <boost/cstdint.hpp>

class matching{
  
  ros::NodeHandle nt;
  
  // Pointcloud Vector
  std::vector<float> xi;
  std::vector<float> zi;

  // Transform Parameters
  float phi;
  float fov_s;
  float fov_e;
  std::vector<float> v0;  
  std::vector<float> v_r;
  
  // Helper to transform each laser scan accordingly
  int h;

  //
  matlab::Engine engi;

public:
  matching() {};
  matching(ros::NodeHandle n_,float phi_,float fov_s_,float fov_e_,std::vector<float> &v0_,int h_);
 // matching(ros::NodeHandle n_,float phi_,float fov_s_,float fov_e_,std::vector<float> v0_,int h_);
 // std::vector<float> getResultVector();
  void matchCallback(const sensor_msgs::PointCloud::ConstPtr& msg);
  void callStairparam();
};


matching::matching(ros::NodeHandle n_,float phi_,float fov_s_,float fov_e_,std::vector<float> &v0_,int h_):
 nt(n_),phi(phi_),fov_s(fov_s),fov_e(fov_e_),v0(v0_),h(h_)
{
  

}

void matching::matchCallback(const sensor_msgs::PointCloud::ConstPtr& msg)
{
  // Initialize Vector transformed around initial guess
  for (int i=0;i<fov_e;++i)
	{
	float a = msg->points[i+fov_s].x;
	float b = msg->points[i+fov_s].y;
	xi.push_back(- b*sin(phi)*h + a*cos(phi));
	zi.push_back(- a*cos(phi) - b*cos(phi)*h);
	};

  //callStairparam();
}

void matching::callStairparam() {
  double test;
 // engine.put("test",test);
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

  // Transform Parameters
  int fov_s;
  int fov_e;
  float phi_1;
  float phi_2;
  
  // Initial Vector
  std::vector<float> v0;

public:
  Angle(ros::NodeHandle n_);
};


Angle::Angle(ros::NodeHandle n_):
n(n_),fov_s(311),fov_e(531),phi_1(-3*3.14/180),phi_2(1*3.14/180)//,cloud_1(n_,phi_1,fov_s,fov_e,v0,1) //,cloud_2(n_,phi_2,811-fov_e,811-fov_s,v0,-1)
{
  // initialize start vector v0
  v0.push_back(.17);
  v0.push_back(.3);
  v0.push_back(.05);
  v0.push_back(.0);
  v0.push_back(10*3.14/180);

  // create & asssign temporary matching object
  matching cloud_1_t(n_,phi_1,fov_s,fov_e,v0,1);
  matching cloud_2_t(n_,phi_2,811-fov_e,811-fov_s,v0,-1);

  cloud_1 = cloud_1_t;
  cloud_2 = cloud_2_t;

  // create subscriber
  sub_1 = n.subscribe("cloud_1",1000, &matching::matchCallback, &cloud_1);
  sub_2 = n.subscribe("cloud_2",1000, &matching::matchCallback, &cloud_2);
}



int main(int argc, char **argv) {

 ros::init(argc,argv,"angle");
 ros::NodeHandle n;
 Angle angle(n);
 ROS_INFO("Woking fine, son!");
 matlab::Engine engine(true);

 double a = 3;
 engine.put("a",a);
 engine.changeWorkingDirectory("src/scalaser/matlab"); // Path to matlab function
 engine.executeCommand("test()");
 ROS_INFO("NOT WORKING");
 engine.openCommandLine();
 ros::spin();

 return 0;
}
