#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float64.h"
#include "math.h"

class ChairModel
{
  ros::NodeHandle n_;
  // Subscriber
  ros::Subscriber stair_sub;
  // Publisher
  ros::Publisher pub_tracks;
  ros::Publisher pub_chair;


  // Markers
  visualization_msgs::Marker marker_tracks;
  visualization_msgs::Marker marker_chair;

  std::vector<double> v_s;
  double beta;
  double phi0;
  double dz0;


public:
  ChairModel(ros::NodeHandle n,std::string frame_id_);
  void initTracks(std::string frame_id);
  void setTracks();
  void initChair(std::string frame_id);
  void setChair();
  void publishModel();
  void stairCallback(const std_msgs::Float64MultiArray::ConstPtr& stair_param);

};

 ChairModel::ChairModel(ros::NodeHandle n,std::string frame_id_): n_(n),v_s(5,0) {
  n_.param("/scalaser/dzi",dz0,.65);
  n_.param("/scalaser/phi",phi0,43*3.14/180);
  phi0 = -phi0*3.14/180;

  initTracks(frame_id_);
  initChair(frame_id_);
  marker_tracks.lifetime = ros::Duration();
  marker_chair.lifetime = ros::Duration();

}


void ChairModel::stairCallback(const std_msgs::Float64MultiArray::ConstPtr& stair_param) {
  v_s.clear();
  for (int i=0;i<5;i++) v_s.push_back(stair_param->data[i]);
}


void ChairModel::initTracks(std::string frame_id_) {
  pub_tracks = n_.advertise<visualization_msgs::Marker>("/tracks_model",1);
  marker_tracks.header.frame_id = frame_id_;
  marker_tracks.header.stamp = ros::Time::now();
  marker_tracks.ns = "chair_model"; 
  marker_tracks.id = 0;
  marker_tracks.type = visualization_msgs::Marker::MESH_RESOURCE;
  marker_tracks.mesh_resource = "package://scalaser/meshes/tracks.stl";
  marker_tracks.mesh_use_embedded_materials = true;
  marker_tracks.action = visualization_msgs::Marker::ADD;
}


void ChairModel::initChair(std::string frame_id_) {
  pub_chair = n_.advertise<visualization_msgs::Marker>("/chair_model",1);
  marker_chair.header.frame_id = frame_id_;
  marker_chair.header.stamp = ros::Time::now();
  marker_chair.ns = "chair_model"; 
  marker_chair.id = 0;
  marker_chair.type = visualization_msgs::Marker::MESH_RESOURCE;
  marker_chair.mesh_resource = "package://scalaser/meshes/chair.stl";
  marker_chair.mesh_use_embedded_materials = true;
  marker_chair.action = visualization_msgs::Marker::ADD;
}


void ChairModel::setTracks() { 

  double h = v_s[0];
  double t = v_s[1];
  double dx = v_s[2];
  double dz = dz0+v_s[3];
  double phi = -phi0-v_s[4];
  double theta = atan(t/h);


  marker_tracks.pose.position.x = 0.125-.65;
  marker_tracks.pose.position.y = 0;
  marker_tracks.pose.position.z = -0.342-.2;

  marker_tracks.pose.orientation.x = 0;
  marker_tracks.pose.orientation.y = sin(.258-phi/2);
  marker_tracks.pose.orientation.z = 0;
  marker_tracks.pose.orientation.w = cos(.258-phi/2);
  // Set Scale
  marker_tracks.scale.x = 0.001;
  marker_tracks.scale.y = 0.001;
  marker_tracks.scale.z = 0.001;

  // // Set colour
  marker_tracks.color.r = 0.7f;
  marker_tracks.color.g = 0.65f;
  marker_tracks.color.b = 0.65f;
  marker_tracks.color.a = 1.0;
}


void ChairModel::setChair() { 

  marker_chair.pose.position.x = - 0.233;
  marker_chair.pose.position.y = 0;
  marker_chair.pose.position.z = -0.185;

  marker_chair.pose.orientation.x = 0;
  marker_chair.pose.orientation.y = 0;
  marker_chair.pose.orientation.z = 0;
  marker_chair.pose.orientation.w = 0;
  // Set Scale
  marker_chair.scale.x = 0.001;
  marker_chair.scale.y = 0.001;
  marker_chair.scale.z = 0.001;

  // Set colour
  marker_chair.color.r = 0.7f;
  marker_chair.color.g = 0.65f;
  marker_chair.color.b = 0.65f;
  marker_chair.color.a = 1.0;


}

void ChairModel::publishModel() {
  pub_tracks.publish(marker_tracks);
  pub_chair.publish(marker_chair);
  // marker_tracks.action = visualization_msgs::Marker::DELETE;
  // marker_chair.action = visualization_msgs::Marker::DELETE;


}

int main( int argc, char** argv ) {
  ros::init(argc,argv,"chair_model");
  ros::NodeHandle n;
  std::string frame_id ("/laser_mount_link");

  ChairModel chair(n,frame_id);

  ros::Rate loop_rate(8); // [Hz]


  while (ros::ok()) {
    chair.setTracks();
    chair.setChair();
    chair.publishModel();
    loop_rate.sleep();
    ros::spinOnce();
  }
}



















