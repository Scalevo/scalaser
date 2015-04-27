#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float32.h"
#include "math.h"

class StairModel
{
  ros::NodeHandle n_;
  // Subscriber
  ros::Subscriber stair_sub;
  ros::Subscriber beta_sub;
  // Publisher
  ros::Publisher stair_pub;
  // Markers
  visualization_msgs::Marker marker;
  
  // Stair Parameters
  std::vector<float> v_s;
  float beta;
  float phi0;
  float dz0;

public:
  StairModel(ros::NodeHandle n,std::string frame_id_);
  void InitMarker(std::string frame_id);
  void SetPosition();
  void PublishModel();
  void StairCallback(const std_msgs::Float32MultiArray::ConstPtr& stair_param);
  void BetaCallback(const std_msgs::Float32::ConstPtr& angle);

};

 StairModel::StairModel(ros::NodeHandle n,std::string frame_id_): n_(n),v_s(5,0),beta(3),phi0(43*3.14/180),dz0(.65)
{

  InitMarker(frame_id_);
  stair_sub = n_.subscribe("/stair_parameters",10, &StairModel::StairCallback, this);
  beta_sub = n_.subscribe("/beta",10, &StairModel::BetaCallback, this);
  marker.lifetime = ros::Duration();
}


void StairModel::InitMarker(std::string frame_id_)
{
  stair_pub = n_.advertise<visualization_msgs::Marker>("/stair_model",1);
  marker.header.frame_id = frame_id_;
  marker.header.stamp = ros::Time::now();
  marker.ns = "stair_model"; 
  marker.id = 0;
  marker.type = visualization_msgs::Marker::CUBE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
}

void StairModel::StairCallback(const std_msgs::Float32MultiArray::ConstPtr& stair_param)
{
  v_s.clear();
  for (int i=0;i<5;i++) v_s.push_back(stair_param->data[i]);
}

void StairModel::BetaCallback(const std_msgs::Float32::ConstPtr& angle)
{
  beta = angle->data;
}

void StairModel::SetPosition()
{


    float h = .17;//v_s[0];
    float t = .28;//v_s[1];
    float dx = v_s[2];
    float dz = dz0+v_s[3];
    float phi = phi0-v_s[4];
    float theta = atan(t/h);// - 8*3.14/180;

  ROS_INFO("STAIR PARAMETERS RECEIVED");
  ROS_INFO("h: %f, t: %f, dx: %f, dz: %f, phi: %f",h,t,dx,dz,phi);
   ROS_INFO("THETA: %F",theta);

 /*
    float x = - sin(phi)*dz - cos(phi)*dx - h*cos(phi)*sin(phi) + 0.01;
    float z = - cos(phi)*dz + sin(phi)*dx - h*cos(phi)*cos(phi) - h/2 + 0.03;
    // Set Position
 
    marker.pose.orientation.x = cos(phi) - sin(phi);
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = sin(phi) + cos(phi);
    marker.pose.orientation.w = beta*3.14/180;
    marker.pose.position.x = cos(theta)*x + sin(theta)*z;
    marker.pose.position.y = 0;
    marker.pose.position.z = -sin(theta)*x + cos(theta)*z;
*/

 
    marker.pose.position.x = -sin(theta)*t/2 + cos(theta)*h/2;
    marker.pose.position.y = 0;
    marker.pose.position.z = -cos(theta)*t/2 - sin(theta)*h/2;
    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = sin(theta/2);
    marker.pose.orientation.z = 0;
    marker.pose.orientation.w = cos(theta/2);

    // Set Scale
    marker.scale.x = h;
    marker.scale.y = 1;
    marker.scale.z = t;

    // Set colour
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    geometry_msgs::Point p;

    for (int i=0;i<4;i++)
    {
    p.x = 0 + h*i;
    p.y = 0;
    p.z = 0 + t*i;
    marker.points.push_back(p);
    }



}

void StairModel::PublishModel()
{
    stair_pub.publish(marker);
}

int main( int argc, char** argv )
{
  ros::init(argc,argv,"stair_model");
  ros::NodeHandle n;
  std::string frame_id ("/stair_middle");
  StairModel stair(n,frame_id);

  ros::Rate loop_rate(5); // [Hz]


  while (ros::ok())
  {
    stair.SetPosition();
    stair.PublishModel();
    loop_rate.sleep();
    ros::spinOnce();
  }
}





















