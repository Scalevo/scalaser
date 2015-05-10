#include "ros/ros.h"
#include "angle.h"

int main(int argc, char **argv) {
 ros::init(argc,argv,"angle");
 ros::NodeHandle n;
 Angle angle(n);
 ros::spin();
 return 0;
}