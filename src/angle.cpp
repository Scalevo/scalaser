#include "ros/ros.h"
#include "sensor_msgs/PointCloud.h"
#include "matlabCppInterface/Engine.hpp"
#include <boost/cstdint.hpp>
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"
#include <math.h>

class matching{
  
  ros::NodeHandle n;

  // Static Engine
  static matlab::Engine engine;

  // Publishers
  ros::Publisher se_r_pub;

  // Pointcloud Vector
  Eigen::VectorXd xi;		// X-Vector of pointcloud transformed around inital values
  Eigen::VectorXd zi;		// Z-Vector of pointcloud transformed around inital values
  Eigen::VectorXd xf;		// Common X-Vector of transformed pointcloud and optimized template
  Eigen::VectorXd zf;		// Z-Vector of transformed
  Eigen::VectorXd z_r;		// Z-Vector of optimized template

  // Transform Parameters
  double phi;
  double dzi;
  int fov_s;
  int fov_d;

  // Initial and Result Vector
  Eigen::VectorXd v0;
  Eigen::VectorXd v_r;

  // Error of fminsearch()
  double se_r;
  
  // Helper to transform each laser scan accordingly
  int h;


public:
  matching() {};
  matching(ros::NodeHandle n_,double phi_,double dzi_,int fov_s_,int fov_d_,std::vector<double> &v0_,int h_);
 // std::vector<float> getResultVector();
  void matchCallback(const sensor_msgs::PointCloud::ConstPtr& msg);
  void matchTemplate();
  void publishSe_r();
  Eigen::VectorXd getV_r() {return v_r;};
  double getDx() {return v_r(2);}
  double getSe_r() {return se_r;}
};

///////////////////////////////////////////////////////////////////////

matlab::Engine matching::engine(true); // Initialize static matlab engine

matching::matching(ros::NodeHandle n_,double phi_,double dzi_,int fov_s_,int fov_d_,std::vector<double> &v0_,int h_):
 n(n_),phi(phi_),dzi(dzi_),fov_s(fov_s_),fov_d(fov_d_),h(h_),xi(fov_d_),zi(fov_d_),xf(fov_d_),zf(fov_d_),z_r(fov_d_),v0(v0_.size()),v_r(v0_.size())
{
  if(engine.good()){ROS_INFO("Engine succesfully initialized.");}
  engine.changeWorkingDirectory("~/catkin_ws/src/scalaser/matlab"); 	// Path to matlab function
  for (int i=0;i<v0_.size();++i) v0(i) = v0_[i];			// Copy std::vector into Eigen::VectorXf
  v_r = v0;
}


void matching::matchCallback(const sensor_msgs::PointCloud::ConstPtr& msg)
{
  // Initialize Vector transformed around initial guess
  for (int i=0;i<fov_d;i++)
  {
   double a = msg->points[i+fov_s].x;
   double b = msg->points[i+fov_s].y;
   //xi.push_back(- b*sin(phi)*h + a*cos(phi));
   //zi.push_back(- a*cos(phi) - b*cos(phi)*h);

   //xi(i) = (- b*sin(phi)*h + a*cos(phi)); // Transform of Matlab
   //zi(i) = (- a*sin(phi) - b*cos(phi)*h);
   xi(i) = (- b*cos(phi)*pow((-1),(h-1)) + a*sin(phi));
   zi(i) = (- a*cos(phi) - b*sin(phi)*pow((-1),(h-1)) + dzi);
  }
  matchTemplate();
}

void matching::matchTemplate()
{
  //ROS_INFO("Matching Template of Cloud_%d",h);
  engine.put("xi",xi);
  engine.put("zi",zi);
  engine.put("h",h);

  //engine.executeCommand("test(xi,zi,h)");
  if (se_r>0.05) v_r = v0;
  engine.put("v0",v_r);
/*
  ROS_INFO("BEFORE FMINSEARCH()");
  ROS_INFO("stair heigth________h = %f",v_r(0)); 
  ROS_INFO("stair depth_________t = %f",v_r(1));
  ROS_INFO("phase offset_______dx = %f",v_r(2));
  ROS_INFO("sensor height______dz = %f",v_r(3));
  ROS_INFO("sensor rotation___phi = %f",v_r(4));
  ROS_INFO("__________________________________");
*/
  engine.executeCommand("[v_r,se_r,z_r,xf,zf] = stairparam(xi,zi,v0,h);");
  engine.get("se_r",se_r);
  engine.get("v_r",v_r);

  engine.get("z_r",z_r);			 
  engine.get("xf",xf);
  engine.get("zf",zf);

  ROS_INFO("RESULT VECTOR AFTER FMINSERACH() OF CLOUD_%d",h);
  ROS_INFO("stair heigth________h = %f",v_r(0)); 
  ROS_INFO("stair depth_________t = %f",v_r(1));
  ROS_INFO("phase offset_______dx = %f",v_r(2));
  ROS_INFO("sensor height______dz = %f",v_r(3));
  ROS_INFO("sensor rotation___phi = %f",v_r(4));


  ROS_INFO("Result of fminsearch(): %f",se_r);
  ROS_INFO("__________________________________");
  ROS_INFO("__________________________________");

  publishSe_r();
}


void matching::publishSe_r()
{
    std_msgs::Float32 se_rM;
    se_rM.data = se_r;
    std::string topic = "/se_r_";
    topic += std::to_string(h);
    se_r_pub = n.advertise<std_msgs::Float32>(topic,1000);
    se_r_pub.publish(se_rM);
}

///////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////


class Angle{

private:
  ros::NodeHandle n;

  // Subscribers
  ros::Subscriber sub_1;
  ros::Subscriber sub_2;
  // Publishers
  ros::Publisher pub_1;		// beta
  ros::Publisher pub_2;		// stairParam

  // Matching objects
  matching cloud_1;
  matching cloud_2;

  // Parameters of Base Transform
  int fov_s;
  int fov_d;
  double phi_1;
  double phi_2;
  double dzi;
  
  // Start Values of fminsearch() Vector
  std::vector<double> v0_1;
  std::vector<double> v0_2;
  double a;


  // Result of fminsearch
  Eigen::VectorXd v_s;

  std_msgs::Float32MultiArray stair_param;
  std_msgs::Float32 beta;

public:
  Angle(ros::NodeHandle n_);
  void computeAngle();
  void computeStair();
};

///////////////////////////////////////////////////////////////////////


Angle::Angle(ros::NodeHandle n_):
n(n_),fov_s(200),fov_d(150),phi_1(-43*3.14/180),phi_2(-43*3.14/180),dzi(.65),a(.7),v_s(5)
{
  // initialize start vector v0_1
  v0_1.push_back(0.17);			// heigth - 		h0
  v0_1.push_back(0.3);			// depth - 		t0
  v0_1.push_back(0.0);			// phase offset - 	dx0
  v0_1.push_back(0.0);			// sensor height - 	dz0
  v0_1.push_back(0*3.14/180);		// sensor rotation - 	phi0

  // initialize start vector v0_1
  v0_2.push_back(0.17);			// heigth - 		h0
  v0_2.push_back(0.3);			// depth - 		t0
  v0_2.push_back(0.0);			// phase offset - 	dx0
  v0_2.push_back(0.0);			// sensor height - 	dz0
  v0_2.push_back(0*3.14/180);		// sensor rotation - 	phi0

  


  ROS_INFO("Start Values for fminsearch:");
  ROS_INFO("stair heigth________h0 = %f",v0_1[0]); 
  ROS_INFO("stair depth_________t0 = %f",v0_1[1]);
  ROS_INFO("phase offset_______dx0 = %f",v0_1[2]);
  ROS_INFO("sensor height______dz0 = %f",v0_1[3]);
  ROS_INFO("sensor rotation___phi0 = %f",v0_1[4]);
 
  // create & asssign temporary matching object
  matching cloud_1_t(n_,phi_1,dzi,fov_s,fov_d,v0_1,1);
  matching cloud_2_t(n_,phi_2,dzi,760-fov_s-fov_d,fov_d,v0_2,2);

  cloud_1 = cloud_1_t;
  cloud_2 = cloud_2_t;

  // create subscriber
  sub_1 = n.subscribe("cloud_1",1, &matching::matchCallback, &cloud_1);
  sub_2 = n.subscribe("cloud_2",1, &matching::matchCallback, &cloud_2);

  // advertise topic
  pub_1 = n.advertise<std_msgs::Float32>("/beta",1000);
  pub_2 = n.advertise<std_msgs::Float32MultiArray>("/stair_parameters",1000);
}

void Angle::computeAngle()
{

  beta.data = 180/3.1415*atan((cloud_2.getDx()-cloud_1.getDx())/a);
  pub_1.publish(beta);

}

void Angle::computeStair()
{
  v_s = (cloud_1.getV_r()+cloud_2.getV_r())/2*cos(beta.data*3.14/180);
  stair_param.data.clear();
  for (int i=0;i<5;i++)  stair_param.data.push_back(v_s(i));
  pub_2.publish(stair_param);
}


int main(int argc, char **argv) {

 ros::init(argc,argv,"angle");
 ros::NodeHandle n;
 Angle angle(n);
 ROS_INFO("Angle method constructed"); 

 ros::Rate loop_rate(3); // [Hz]


  while (ros::ok())
  {
    angle.computeAngle();
    angle.computeStair();
    loop_rate.sleep();
    ros::spinOnce();
  }

 return 0;
}
