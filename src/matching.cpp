#include "matching.h"

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
  if (se_r<0.008) v0 = v_r;
  engine.put("v0",v0);
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
/*
  ROS_INFO("RESULT VECTOR AFTER FMINSERACH() OF CLOUD_%d",h);
  ROS_INFO("stair heigth________h = %f",v_r(0)); 
  ROS_INFO("stair depth_________t = %f",v_r(1));
  ROS_INFO("phase offset_______dx = %f",v_r(2));
  ROS_INFO("sensor height______dz = %f",v_r(3));
  ROS_INFO("sensor rotation___phi = %f",v_r(4));


  ROS_INFO("Result of fminsearch(): %f",se_r);
  ROS_INFO("__________________________________");
  ROS_INFO("__________________________________");
*/

if(se_r > .008) ROS_WARN("Pointcloud has not been matched properly. Error: %f",se_r);

  publishSe_r();
}


void matching::publishSe_r()
{
    std_msgs::Float64 se_rM;
    se_rM.data = se_r;
    std::string topic = "/se_r_";
    topic += std::to_string(h);
    se_r_pub = n.advertise<std_msgs::Float64>(topic,1000);
    se_r_pub.publish(se_rM);
}
