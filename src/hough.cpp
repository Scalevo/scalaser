#include "ros/ros.h"
#include "sensor_msgs/PointCloud.h"
#include "laser_geometry/laser_geometry.h"
#include "geometry_msgs/Point32.h"
#include "std_msgs/Float32.h"

class cloudSub{

private:

  ros::NodeHandle n_;
  ros::Publisher lines_pub_;
  std_msgs::float32 delta;

public:

  cloudSub(ros::NodeHandle n) : n_(n)
{
  lines_pub_ = n_.advertise<geometry_msgs::Point32>("beta",1000);
 
}

void cloudCallback(const sensor_msgs::PointCloud::ConstPtr& msg)
{
  geometry_msgs::Point32 edge;
  edge.x = msg->points[1].x;
  edge.y = msg->points[1].y;
  edge = manipEdge(edge);
  lines_pub_.publish(edge);
} 

geometry_msgs::Point32 manipEdge( geometry_msgs::Point32 edge)
{
  edge.x = edge.x+100;
  edge.y = edge.y-100;
  return edge;
}

geometry_msgs::Point32 matching( geometry_msgs::Point32 edge)
{
  


}

sensor_msgs::PointCloud getMsgs () { return msg_; }

};




int main(int argc, char **argv)
{

  ros::init(argc,argv,"hough");
  ros::NodeHandle n;
  cloudSub go_1(n);
  cloudSub go_2(n);
  ros::Subscriber sub_1 = n.subscribe("cloud_1",1000, &cloudSub::cloudCallback,&go_1);
  ros::Subscriber sub_2 = n.subscribe("cloud_2",1000, &cloudSub::cloudCallback,&go_2);
  ros::Publisher main_pub;
  main_pub = n.advertise<sensor_msgs::PointCloud>("cloud_test",1000);
  main_pub.publish(go_1.getMsgs);
  ros::spin();

  return 0;

}
