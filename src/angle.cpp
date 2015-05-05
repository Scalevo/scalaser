#include "angle.h"
//#include "matching.h"

Angle::Angle(ros::NodeHandle n_):
n(n_),a(.7),v_s(5)
{
  
  n.param("/scalaser/fov_s",fov_s,200);
  n.param("/scalaser/fov_d",fov_d,150);
  n.param("/scalaser/dzi",dzi,.65);
  n.param("/scalaser/phi",phi0,-43*3.14/180);
  phi0 = phi0*3.14/180;


ROS_INFO("Phi0: %f",phi0);

  ROS_INFO("Field of View Start: %d",fov_s);
  ROS_INFO("Field of View Window: %d",fov_d); 
  ROS_INFO("Dz Initial: %f",dzi);   

  // initialize start vector v0
  v0.push_back(0.17);			// heigth - 		h0
  v0.push_back(0.3);			// depth - 		t0
  v0.push_back(0.0);			// phase offset - 	dx0
  v0.push_back(0.0);			// sensor height - 	dz0
  v0.push_back(0*3.14/180);		// sensor rotation - 	phi0

  ROS_INFO("Start Values for fminsearch:");
  ROS_INFO("stair heigth________h0 = %f",v0[0]); 
  ROS_INFO("stair depth_________t0 = %f",v0[1]);
  ROS_INFO("phase offset_______dx0 = %f",v0[2]);
  ROS_INFO("sensor height______dz0 = %f",v0[3]);
  ROS_INFO("sensor rotation___phi0 = %f",v0[4]);
 
  // create & asssign temporary matching object
  matching cloud_1_t(n_,phi0,dzi,fov_s,fov_d,v0,1);
  matching cloud_2_t(n_,phi0,dzi,811-fov_s-fov_d,fov_d,v0,2);

  cloud_1 = cloud_1_t;
  cloud_2 = cloud_2_t;

  // create subscriber
  sub_1 = n.subscribe("cloud_1",1, &matching::matchCallback, &cloud_1);
  sub_2 = n.subscribe("cloud_2",1, &matching::matchCallback, &cloud_2);

  // advertise topic
  pub_1 = n.advertise<std_msgs::Float64>("/beta",1000);
  pub_2 = n.advertise<std_msgs::Float64MultiArray>("/stair_parameters",1000);
}

void Angle::computeAngle()
{

  beta.data = 180/3.1415*atan((cloud_2.getDx()-cloud_1.getDx())/a);
  pub_1.publish(beta);

}

void Angle::computeStair()
{
  v_s = (cloud_1.getV_r()+cloud_2.getV_r())/2*cos(beta.data*3.14/180);
  stair_param.data.clear();
  for (int i=0;i<5;i++)  stair_param.data.push_back(v_s(i));
  pub_2.publish(stair_param);
}

void Angle::SetPosition()
{

    double h = v_s(0);
    double t = v_s(1);
    double dx = -v_s(2);
    double dz = dzi+v_s(3);
    double phi = -phi0-v_s(4);
    double theta = atan(t/h);



  transform.setOrigin( tf::Vector3(-(cos(phi)*dx + sin(phi)*dz),0,-(-sin(phi)*dx + cos(phi)*dz)) );
  tf::Quaternion q;
  q.setRPY(0,phi,-beta.data*3.14/180);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform,ros::Time::now(),"laser_mount_link","stair_middle"));
  ROS_INFO_ONCE("POSITION SET");
}

