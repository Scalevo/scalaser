#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "sensor_msgs/PointCloud.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include "laser_geometry/laser_geometry.h"

class LaserScanToPointCloud{

public:

  ros::NodeHandle n_;
  laser_geometry::LaserProjection projector_;
  tf::TransformListener listener_;
  message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub_;
  tf::MessageFilter<sensor_msgs::LaserScan> laser_notifier_;
  ros::Publisher scan_pub_;
  std::string laser_frame_id_;

  LaserScanToPointCloud(ros::NodeHandle n, std::string laser_frame_id_ = "laser", std::string scan_name_ = "scan") : 
    n_(n),
    laser_sub_(n_, scan_name_, 10),
    laser_notifier_(laser_sub_,listener_, laser_frame_id_, 10)
  {
    laser_notifier_.registerCallback(
      boost::bind(&LaserScanToPointCloud::scanCallback, this, _1, laser_frame_id_));
    laser_notifier_.setTolerance(ros::Duration(0.001));
    scan_pub_ = n_.advertise<sensor_msgs::PointCloud>("/cloud",1);
  }

  void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in, std::string laser_frame_id)
  {
    sensor_msgs::PointCloud cloud;
    try
    {
      projector_.transformLaserScanToPointCloud(
          laser_frame_id, *scan_in, cloud, listener_);
    }
    catch (tf::TransformException& e)
    {
        std::cout << e.what();
        return;
    }
    
    // Do something with cloud.

    scan_pub_.publish(cloud);

  }
};

int main(int argc, char** argv)
{
  std::string laser_frame_id = argv[1];
  std::string scan_name = argv[2];
  ros::init(argc, argv, "my_scan_to_cloud");
  ros::NodeHandle n;
  LaserScanToPointCloud lstopc(n,laser_frame_id,scan_name);
  
  ros::spin();
  
  return 0;
}
