#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float32.h"
#include "math.h"

class ChairModel
{
  ros::NodeHandle n_;
  // Publisher
  ros::Publisher chair_pub;
  // Markers
  visualization_msgs::Marker marker;
  
public:
  ChairModel(ros::NodeHandle n,std::string frame_id_);
  void InitMarker(std::string frame_id);
  void SetPosition();
  void PublishModel();

};

 ChairModel::ChairModel(ros::NodeHandle n,std::string frame_id_): n_(n)
{
  InitMarker(frame_id_);
  marker.lifetime = ros::Duration();
}


void ChairModel::InitMarker(std::string frame_id_)
{
  chair_pub = n_.advertise<visualization_msgs::Marker>("/chair_model",1);
  marker.header.frame_id = frame_id_;
  marker.header.stamp = ros::Time::now();
  marker.ns = "chair_model"; 
  marker.id = 0;
  marker.type = visualization_msgs::Marker::MESH_RESOURCE;
  marker.mesh_resource = "package://scalaser/meshes/scalevo.stl";
  marker.action = visualization_msgs::Marker::ADD;
}


void ChairModel::SetPosition()
{ 
    marker.pose.position.x = .42;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 1;
    marker.pose.orientation.z = 0;

    // Set Scale
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;

    // Set colour
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;





}

void ChairModel::PublishModel()
{
    chair_pub.publish(marker);
}

int main( int argc, char** argv )
{
  ros::init(argc,argv,"chair_model");
  ros::NodeHandle n;
  std::string frame_id ("/laser_mount_link");
  ChairModel chair(n,frame_id);

  ros::Rate loop_rate(5); // [Hz]


  while (ros::ok())
  {
    chair.SetPosition();
    chair.PublishModel();
    loop_rate.sleep();
    ros::spinOnce();
  }
}





















