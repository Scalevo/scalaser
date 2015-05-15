#include <ros/ros.h>
#include "stair_model.h"

int main( int argc, char** argv )
{
  ros::init(argc,argv,"stair_model");
  ros::NodeHandle n;
  std::string frame_id ("/stair_middle");
  StairModel stair(n,frame_id);

  ros::Rate loop_rate(2); // [Hz]
  while (ros::ok())
  {
    stair.setPosition();
    stair.publishModel();
    loop_rate.sleep();
    ros::spinOnce();
  }
}





















