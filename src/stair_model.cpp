#include "stair_model.h"

 StairModel::StairModel(ros::NodeHandle n,std::string frame_id_): n_(n), frame_id(frame_id_), v_s(5,0), beta(0), stair_bot(0), stair_top(0)
{
  n_.param("/scalaser/dzi",dz0,.65);
  n_.param("/scalaser/phi",phi0,43*3.14/180);
  phi0 = -phi0*3.14/180;

  ROS_INFO("dzi: %f",dz0);
  ROS_INFO("phi: %f",phi0);
  stair_sub = n_.subscribe("/stair_parameters",10, &StairModel::stairCallback, this);
  beta_sub = n_.subscribe("/beta",10, &StairModel::betaCallback, this);
}


void StairModel::initMarker(std::string frame_id_)
{
  stair_pub = n_.advertise<visualization_msgs::Marker>("/stair_model",1);
  marker.header.frame_id = frame_id_;
  marker.header.stamp = ros::Time::now();
  marker.ns = "stair_model"; 
  marker.id = 0;
  marker.type = visualization_msgs::Marker::CUBE_LIST;
  marker.lifetime = ros::Duration();
}

void StairModel::stairCallback(const std_msgs::Float64MultiArray::ConstPtr& stair_param)
{
  v_s.clear();
  for (int i=0;i<5;i++) v_s.push_back(stair_param->data[i]);
}

void StairModel::betaCallback(const std_msgs::Float64::ConstPtr& angle)
{
  beta = angle->data;
}

void StairModel::setPosition() {
  initMarker(frame_id);
  marker.action = visualization_msgs::Marker::ADD;
  marker.points.clear();


  double h = v_s[0];
  double t = v_s[1];
  double dx = v_s[2];
  double dz = dz0+v_s[3];
  double phi = -phi0-v_s[4];
  double theta = atan(t/h);// - 8*3.14/180;
  double diag = sqrt(h*h+t*t);
  /*
  ROS_INFO("STAIR PARAMETERS RECEIVED");
  ROS_INFO("h: %f, t: %f, dx: %f, dz: %f, phi: %f",h,t,dx,dz,phi);
  ROS_INFO("THETA: %F",theta);
  */
  /*
  double x = - sin(phi)*dz - cos(phi)*dx - h*cos(phi)*sin(phi) + 0.01;
  double z = - cos(phi)*dz + sin(phi)*dx - h*cos(phi)*cos(phi) - h/2 + 0.03;
  // Set Position

  marker.pose.orientation.x = cos(phi) - sin(phi);
  marker.pose.orientation.y = 0;
  marker.pose.orientation.z = sin(phi) + cos(phi);
  marker.pose.orientation.w = beta*3.14/180;
  marker.pose.position.x = cos(theta)*x + sin(theta)*z;
  marker.pose.position.y = 0;
  marker.pose.position.z = -sin(theta)*x + cos(theta)*z;
  */

  marker.pose.position.x = -sin(theta)*t/2 + cos(theta)*h/2 + sin(theta)*t;
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

  ROS_INFO("Dx: %f",dx);
  ROS_INFO("Diag: %f",diag);
  ROS_INFO("dx / diagonal: %f",dx/diag);

  if (dx/diag < stair_bot) {
    stair_bot = dx/diag;
    ROS_INFO("Stair Bottom: %f", stair_bot);
  }
  else if (dx/diag > stair_top) {
    stair_top = dx/diag;
    ROS_INFO("Stair Top: %f", stair_top);
  }

  geometry_msgs::Point p;
  for (int i=floor(stair_bot); i < ceil(stair_top)+3; i++) {
  p.x = 0 - h*i;
  p.y = 0;
  p.z = 0 - t*i;
  marker.points.push_back(p);
  }

}

void StairModel::publishModel()
{
  stair_pub.publish(marker);
  marker.action = visualization_msgs::Marker::DELETE;
}


