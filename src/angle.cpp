#include "angle.h"

Angle::Angle(ros::NodeHandle n_):
n(n_),a(.7),v_s(5),beta_old(0),v0{0.17,0.3,0.0,0.0,0.0}
{
  setParameters();

  phi0 = phi0*3.14/180;

  ROS_INFO("Phi0: %f",phi0);
  ROS_INFO("Field of View Start: %d",fov_s);
  ROS_INFO("Field of View Window: %d",fov_d);
  ROS_INFO("Dz Initial: %f",dzi);

  // ROS_INFO("Start Values for fminsearch:");
  // ROS_INFO("stair heigth________h0 = %f",v0[0]); 
  // ROS_INFO("stair depth_________t0 = %f",v0[1]);
  // ROS_INFO("phase offset_______dx0 = %f",v0[2]);
  // ROS_INFO("sensor height______dz0 = %f",v0[3]);
  // ROS_INFO("sensor rotation___phi0 = %f",v0[4]);

  // create & asssign temporary matching object
  matching cloud_1_t(n_,phi0,dzi,fov_s,fov_d,v0,1);
  matching cloud_2_t(n_,phi0,dzi,811-fov_s-fov_d,fov_d,v0,2);

  cloud_1 = cloud_1_t;
  cloud_2 = cloud_2_t;

  main_timer = n.createTimer(ros::Duration(0.25),&Angle::timerCallback,this,false);

  // initialize service
  ros::ServiceServer service = n.advertiseService("align_wheelchair",&Angle::alignWheelchair,this);

  ROS_INFO("Service started. Waiting for input.");

  ros::spin();
}

bool Angle::alignWheelchair(scalevo_msgs::Starter::Request& request, scalevo_msgs::Starter::Response& response) {
  if (request.on) {
    // create subscriber
    sub_1 = n.subscribe("cloud_1",1, &matching::matchCallback, &cloud_1);
    sub_2 = n.subscribe("cloud_2",1, &matching::matchCallback, &cloud_2);

    // advertise topics
    pub_1 = n.advertise<std_msgs::Float64>("/beta",100);
    pub_2 = n.advertise<std_msgs::Float64MultiArray>("/stair_parameters",100);
    pub_velocity = n.advertise<std_msgs::Float64MultiArray>("/velocity",100);

    // start main loop timer
    main_timer.start();

    ROS_INFO("Wheelchair alignment has been started.");
    return true;
  }

  else if (!request.on) {
    // shutdown subscribers
    sub_1.shutdown();
    sub_2.shutdown();
    
    // shutdown publishers
    pub_1.shutdown();
    pub_2.shutdown();
    pub_velocity.shutdown();

    // stop main loop timer
    main_timer.stop();

    ROS_INFO("Wheelchair alignment has been stopped.");
    return true;
  }
  return true;
}

void Angle::timerCallback(const ros::TimerEvent& event)
{
  if (sub_1.getNumPublishers() > 0 && sub_2.getNumPublishers() > 0) {
    cloud_1.setData();
    cloud_2.setData();
    cloud_1.matchTemplate();
    cloud_2.matchTemplate();
    computeAngle();
    computeStair();
    computeVelocity();
    setPosition();
    ROS_INFO("Callback time: %f",event.profile.last_duration.toSec());
  }
}

void Angle::computeAngle()
{
  beta_new = 180/3.1415*atan((cloud_2.getDx()-cloud_1.getDx())/a);
 
  if ( fabs(beta_old - beta_new) < 10)
    {
      beta.data = beta_new;
      pub_1.publish(beta);
    }
  else {
    ROS_WARN("No beta published since unreasonable values occured.");
    ROS_WARN("Refused beta: %f",beta_new);
  }
  beta_old = beta.data;
}

void Angle::computeStair()
{
  v_s = (cloud_1.getV_r()+cloud_2.getV_r())/2*cos(beta.data*3.14/180);
  stair_param.data.clear();
  for (int i=0;i<5;i++)  stair_param.data.push_back(v_s(i));
  pub_2.publish(stair_param);
}

void Angle::computeVelocity()
{
  n.param("/scalaser/kp",kp,10.0);
  velocity.data.clear();
  velocity.data.push_back(0);
  if(cloud_1.getSe_r()<threshold && cloud_2.getSe_r()<threshold)
    {velocity.data.push_back(-beta.data*3.1415/180*kp);}
  else {
    velocity.data.push_back(0);
    ROS_WARN("No velocity published since matching didn't work properly.");
  }

  pub_velocity.publish(velocity);
}



void Angle::setPosition()
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
  ROS_INFO_ONCE("Transform to stair sent");
}

void Angle::setParameters()
{
  n.param("/scalaser/fov_s",fov_s,200);
  n.param("/scalaser/fov_d",fov_d,150);
  n.param("/scalaser/dzi",dzi,.65);
  n.param("/scalaser/phi",phi0,-43*3.14/180);
  n.param("/scalaser/kp",kp,10.0);
  n.param("/scalaser/threshold",threshold,0.08);
}