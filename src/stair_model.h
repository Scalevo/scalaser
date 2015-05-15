#ifndef STAIR_MODEL_H
#define STAIR_MODEL_H

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float64.h"
#include "math.h"

class StairModel
{
private:
  ros::NodeHandle n_;
  // Subscriber
  ros::Subscriber stair_sub;
  ros::Subscriber beta_sub;
  // Publisher
  ros::Publisher stair_pub;
  // Markers
  visualization_msgs::Marker marker;
  // Tf frame
  std::string frame_id;
  
  // Stair Parameters
  std::vector<double> v_s;
  double beta;
  double phi0;
  double dz0;

  // stair viz parameters
  double stair_bot;
  double stair_top;

public:
  StairModel(ros::NodeHandle n,std::string frame_id_);
  void initMarker(std::string frame_id);
  void setPosition();
  void publishModel();
  void stairCallback(const std_msgs::Float64MultiArray::ConstPtr& stair_param);
  void betaCallback(const std_msgs::Float64::ConstPtr& angle);

};


#endif
