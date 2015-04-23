#include "ros/ros.h"
#include "sensor_msgs/PointCloud.h"
#include "matlabCppInterface/Engine.hpp"
#include <boost/cstdint.hpp>
#include <math.h>

class Test
{// this is static since we open multiple instances of the class
  static matlab::Engine engine;
  Eigen::VectorXd inputV;
  Eigen::VectorXd outputV;

public:
  Test();
  void callFunc();

};
// inputV(4),outputV(4) -> Initialize vector to desired size
Test::Test(): inputV(4),outputV(4) {
  engine.initialize();
  for (int i = 0;i<4;i++) {		// Fill Vector with values
  inputV(i) = i;
  }

  engine.changeWorkingDirectory("~/catkin_ws/src/scalaser/matlab"); 	// Path to
}

void Test::callFunc(){

  engine.put("inputV",inputV);
// Execute a simple Test Function and save the result in to outputV		
  engine.executeCommand("[outputV]=test_simple(inputV)"); 
  engine.get("outputV",outputV);
  for (int i = 0;i<4;i++) ROS_INFO("Intput: %f ---- Output: %f",inputV(i),outputV(i));
}

matlab::Engine Test::engine(true); // Initialize static matlab engine

int main(int argc, char **argv) {

 ros::init(argc,argv,"angle");
 ros::NodeHandle n;
 Test testO;


 // Run Test Function at desired frequency
 ros::Rate loop_rate(1); // [Hz]

  while (ros::ok())
  {
    testO.callFunc();
    loop_rate.sleep();
    ros::spinOnce();
  }

 return 0;
}
