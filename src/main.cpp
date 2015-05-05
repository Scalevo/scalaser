#include "ros/ros.h"
#include "angle.h"


int main(int argc, char **argv) {

 ros::init(argc,argv,"angle");
 ros::NodeHandle n;
 Angle angle(n);
 ROS_INFO("Angle method constructed"); 
 ros::Rate loop_rate(10); // [Hz]
  while (ros::ok())
  {
    angle.computeAngle();
    angle.computeStair();
    angle.SetPosition();
    loop_rate.sleep();
    ros::spinOnce();
  }

 return 0;
}
