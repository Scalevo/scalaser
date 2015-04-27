#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"

class Laser2Stair
{
  ros::NodeHandle n_;
  // Subscriber
  ros::Subscriber stair_sub;
  ros::Subscriber beta_sub;

  // Tf
  tf::TransformBroadcaster br;
  tf::Transform transform;
  
  // Stair Parameters
  std::vector<float> v_s;
  float beta;
  float phi0;
  float dz0;
  float h;
  float t;
  float dx;
  float dz;
  float phi;

public:
  Laser2Stair(ros::NodeHandle n);
  void SetPosition();
  void StairCallback(const std_msgs::Float32MultiArray::ConstPtr& stair_param);
  void BetaCallback(const std_msgs::Float32::ConstPtr& angle);

};


 Laser2Stair::Laser2Stair(ros::NodeHandle n): n_(n),v_s(5,0),beta(3),phi0(43*3.14/180),dz0(.65)
{
  stair_sub = n_.subscribe("/stair_parameters",10, &Laser2Stair::StairCallback, this);
  beta_sub = n_.subscribe("/beta",10, &Laser2Stair::BetaCallback, this);
  ROS_INFO("CLASS CONSTRUCTED");
}

void Laser2Stair::StairCallback(const std_msgs::Float32MultiArray::ConstPtr& stair_param)
{
  v_s.clear();
  for (int i=0;i<5;i++) v_s.push_back(stair_param->data[i]);
  h = v_s[0];
  t = v_s[1];
  dx = v_s[2];
  dz = dz0+v_s[3];
  phi = phi0-v_s[4];

  ROS_INFO("STAIR PARAMETERS RECEIVED");
  ROS_INFO("h: %f, t: %f, dx: %f, dz: %f, phi: %f",h,t,dx,dz,phi);
}

void Laser2Stair::BetaCallback(const std_msgs::Float32::ConstPtr& angle)
{
  beta = angle->data;
 // ROS_INFO("BETA RECEIVED");
}

void Laser2Stair::SetPosition()
{
  transform.setOrigin( tf::Vector3(-(cos(phi)*dx + sin(phi)*dz),0,-(-sin(phi)*dx + cos(phi)*dz)) );
  tf::Quaternion q;
  q.setRPY(0,phi,-beta*3.14/180);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform,ros::Time::now(),"laser_mount_link","stair_middle"));
  //ROS_INFO("POSITION SET");
}

int main(int argc, char** argv){
  ros::init(argc, argv, "laser2stair_tf");
  ros::NodeHandle n;
  Laser2Stair laser2stair(n);

  ros::Rate rate(10.0);
  while (n.ok()){
    laser2stair.SetPosition();
    ros::spinOnce();
  }

  return 0;
};





