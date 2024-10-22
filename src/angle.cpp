#include "angle.h"

Angle::Angle(ros::NodeHandle n_):
n(n_), a(.7), v_s(5), beta_old(0), beta_new(0), v0(5), count(0), wrong_beta_count(0), wrong_se_r_count(0)
r_h(.1016), s(.653837), phi_f(.193897),
kp(0), vel_fwd(0),
v_r_1(2), v_r_2(2),
alpha(0)
{
  setParameters();
  initializePlotEngine();
  initializeEdge();


  v0 << 0.17, 0.3, 0.0, 0.0, 0.0;

  // ROS_INFO("Start Values for fminsearch:");
  // ROS_INFO("stair heigth________h0 = %f",v0(0); 
  // ROS_INFO("stair depth_________t0 = %f",v0(1);
  // ROS_INFO("phase offset_______dx0 = %f",v0(2);
  // ROS_INFO("sensor height______dz0 = %f",v0(3);
  // ROS_INFO("sensor rotation___phi0 = %f",v0(4);

  // create & asssign temporary matching object
  matching cloud_1_t(n_,phi0,dzi,fov_s,fov_d,v0,1);
  matching cloud_2_t(n_,phi0,dzi,811-fov_s-fov_d,fov_d,v0,2);
  cloud_1 = cloud_1_t;
  cloud_2 = cloud_2_t;

  // initialize main timer
  main_timer = n.createTimer(ros::Duration(0.2),&Angle::timerCallback,this,false,false);

  // initialize service
  service = n.advertiseService("align_wheelchair",&Angle::alignWheelchair,this);

  ROS_INFO("Node started. Waiting for input.");

  ros::spin();
}

bool Angle::alignWheelchair(scalevo_msgs::Starter::Request& request, scalevo_msgs::Starter::Response& response) {
  if (request.on) {

    // create subscriber
    sub_1 = n.subscribe("scan_1", 1, &matching::matchCallback, &cloud_1);
    sub_2 = n.subscribe("scan_2", 1, &matching::matchCallback, &cloud_2);

    sub_joint = n.subscribe("/joint_states", 1, &Angle::jointCallback, this);


    // advertise topics
    pub_1 = n.advertise<std_msgs::Float64>("/beta", 1);
    pub_2 = n.advertise<std_msgs::Float64MultiArray>("/stair_parameters", 1);
    // pub_velocity = n.advertise<std_msgs::Float64MultiArray>("/set_vel", 100);
    pub_s_velocity = n.advertise<std_msgs::String>("/scalevo_cmd", 1);
    pub_edge_marker = n.advertise<visualization_msgs::Marker>("edge_marker", 1);

    while (sub_1.getNumPublishers() == 0 || sub_2.getNumPublishers() == 0) {
      sleep(1);
    }

    ros::spinOnce();
    
    initializeMatching();
    
    // start main loop timer
    main_timer.start();

    ROS_INFO("Wheelchair alignment has been started.");
    return true;  

  }

  else if (!request.on) {
    // shutdown subscribers
    sub_1.shutdown();
    sub_2.shutdown();
    sub_joint.shutdown();
    
    // shutdown publishers
    pub_1.shutdown();
    pub_2.shutdown();
    // pub_velocity.shutdown();
    pub_s_velocity.shutdown();
    pub_edge_marker.shutdown();

    // stop main loop timer
    main_timer.stop();

    ROS_INFO("Nr of computations: %d", count);
    ROS_INFO("Duration:           %f", ros::Time::now().toSec() - time_start);
    ROS_INFO("Average frequency:  %f Hz",count/(ros::Time::now().toSec() - time_start));
    ROS_INFO("Wheelchair alignment has been stopped.");

    plotData(beta_vector);
    plot_engine.executeCommand("savefig(datestr(now))");
    ROS_INFO("Plot has been saved to file.");

    return true;
  }
  return true;
}

void Angle::initializePlotEngine() {
  // plot data in a new matlab engine
  if (plot_engine.initialize() && plot_engine.good()) ROS_INFO("Plot engine succesfully initialized.");
  plot_engine.changeWorkingDirectory("~/catkin_ws/src/scalaser/matlab");
  // plot_engine.showWorkspace();
  plotData(beta_vector);

}

void Angle::timerCallback(const ros::TimerEvent& event) {
  if (sub_1.getNumPublishers() > 0 && sub_2.getNumPublishers() > 0) {

    cloud_1.setData();
    cloud_2.setData();
    v_r_1 = cloud_1.matchTemplate();
    v_r_2 = cloud_2.matchTemplate();

    if (cloud_1.getSe_r() < threshold && cloud_2.getSe_r() < threshold) {
      computeAngle();
      computeStair();
      pubStairParameters();
      pubEdge();
    }
    else {
      ROS_WARN("Nothing published because fmincon did not converge.");
    }

    computeVelocity();

    ROS_INFO("Callback time:        %f",event.profile.last_duration.toSec());
    count++;
  }

 //  if (fmod(count,5) == 0) {
 //    plotData(beta_vector);
 // }
}

void Angle::jointCallback(const sensor_msgs::JointState::ConstPtr& joint_state) {
  // phi0 = -joint_state -> position[0];
  // dzi = r_h + s*sin(-phi0 + phi_f);

  // to only set parameters after reinitialization comment this code and write phi0 and dzi to the parameter server instead
  // cloud_1.setParameters(phi0, dzi, fov_s, ofv_d);
  // cloud_2.setParameters(phi0, dzi, 811-fov_s-fov_d, fov_d);
  // ros::param::set("/scalaser/phi", PI/180*phi0);
  // ros::param::set("/scalaser/dzi", dzi);

  // ROS_INFO("Matching Parameters have been updated.");
}

void Angle::initializeMatching() {

  setParameters();

  beta_new = 0;
  beta_old = 0;
  beta_vector.clear();
  alpha_vector.clear();
  dx_1_vector.clear();
  dx_2_vector.clear();
  time_vector.clear();

  count = 0;

  cloud_1.setParameters(phi0,dzi,fov_s,fov_d);
  cloud_2.setParameters(phi0,dzi,811-fov_s-fov_d,fov_d);


  ROS_INFO("Start time: %f",time_start);
  
  setBoundaries();

  time_start = ros::Time::now().toSec();
  count++;
}

void Angle::setBoundaries() {
  cloud_1.setData();
  cloud_2.setData();
  cloud_1.setFminArgs(v0);
  v_r_1 = cloud_1.matchTemplate();
  cloud_2.setFminArgs(v_r_1);
  v_r_2 = cloud_2.matchTemplate();
}

void Angle::computeAngle() {

  dx_1 = cloud_1.getDx();
  dx_2 = cloud_2.getDx();
  diag_1 = cloud_1.getDiag();
  diag_2 = cloud_2.getDiag();

// Experimental for Curve
  if (fabs(alpha) > 1) {
    if (fabs(dx_1) > 0.8*diag_1 || fabs(dx_2) > 0.8*diag_2) {
      v0 = cloud_1.getV_r();
      v0(2) = 0;
      setBoundaries();
      dx_1 = cloud_1.getDx();
      dx_2 = cloud_2.getDx();
      diag_1 = cloud_1.getDiag();
      diag_2 = cloud_2.getDiag();
      ROS_INFO("------Edge has been changed!------");
    }
  }
  computeAlpha();
  // updateFoV();
  // setFoV();
  // setDx();
// Endperimental

  computeBeta();
}

void Angle::computeAlpha() {
  // Get Parameters from matching objects
  alpha_1 = atan((dx_2 - dx_1) / a);
  alpha_2 = atan(((diag_2 - dx_2) - (diag_1 - dx_1)) / a);
  alpha = 180/PI*(alpha_1 + alpha_2);
  
  // ROS_INFO("ALPHA_1: %f°", 180/PI*(alpha_1));
  // ROS_INFO("ALPHA_2: %f°", 180/PI*(alpha_2));
  // ROS_INFO("ALPHA: %f°", alpha);
  // ROS_INFO("BETAA: %f°", beta_new);
  // ROS_INFO("BETAB: %f°", 180/PI*(alpha_1));
}

void Angle::computeBeta() {

  // beta_new = 180/PI*atan((dx_2 - dx_1)/a); // Single Edge Beta Computation

  beta_new = 180/PI*(alpha_1 - alpha_2)/2;
  ROS_INFO("ALPHA:         %f°", alpha);
  ROS_INFO("BETA_SIMPLE:   %f°", beta_new);

    // Linear interpolation between two edges
  d_beta = alpha*(dx_2 + dx_1)/(diag_2 + diag_1) - alpha/2;
  beta_new -= d_beta; // + or - not sure yet

  ROS_INFO("D_BETA:        %f°", d_beta);
  ROS_INFO("BETA_INTERP:   %f°", beta_new);

  // if (fabs(beta_old - beta_new) < 15 && fabs(beta_old) < 10) {
  if (fabs(beta_new) < 15) {

    beta.data = beta_new;
    pub_1.publish(beta);

    ROS_INFO("Time: %f",ros::Time::now().toSec()-time_start);
    time_vector.push_back(ros::Time::now().toSec()-time_start);
    alpha_vector.push_back(180/PI*(alpha_1 + alpha_2));
    beta_vector.push_back(beta_new);

    v0 = cloud_1.getV_r();
    dx_1_vector.push_back(v0(2));
    v0 = cloud_2.getV_r();
    dx_2_vector.push_back(v0(2));

    wrong_beta_count = 0;
  }
  else {
    wrong_beta_count++;
    ROS_WARN("No beta published since unreasonable values occured. ⁻ %d",wrong_beta_count);
    ROS_WARN("Refused beta: %f",beta_new);
    if (wrong_beta_count > 3) initializeMatching();
  }
  beta_old = beta.data;
}

void Angle::computeStair() {
  v_s = (cloud_1.getV_r()+cloud_2.getV_r())/2*cos(beta.data*PI/180);
  stair_param.data.clear();
  for (int i=0; i < 5; i++)  stair_param.data.push_back(v_s(i));
  pub_2.publish(stair_param);
}

void Angle::computeVelocity() {

  // Read Velocity Parameters from Server
  n.param("/scalaser/kp",kp,0.05);
  n.param("/scalaser/vel_fwd",vel_fwd,0.0);

  // velocity.data.clear();
  // velocity.data.push_back(0);
  // if(cloud_1.getSe_r()<threshold && cloud_2.getSe_r()<threshold)
  //   {velocity.data.push_back(-beta.data*PI/180*kp);}
  // else {
  //   velocity.data.push_back(0);
  //   ROS_WARN("No velocity published since matching didn't work properly.");
  // }

  // Fill velo message
  std_msgs::String velo;

  velo.data = "set_vel,"; 
  std::ostringstream buff_1;
  buff_1 << vel_fwd;
  velo.data += buff_1.str();
  velo.data += ","; 
  std::ostringstream buff_2;

  if(cloud_1.getSe_r() < threshold && cloud_2.getSe_r() < threshold) {
    buff_2 << - beta.data * kp;
    velo.data += buff_2.str();
    wrong_se_r_count = 0;
  }
  else {
    wrong_se_r_count++;
    buff_2 << 0;
    velo.data += buff_2.str();
    if (wrong_se_r_count > 3) initializeMatching();
  }
  pub_s_velocity.publish(velo);
}

void Angle::pubStairParameters() {

  double h = v_s(0);
  double t = v_s(1);
  double dx = -v_s(2);
  double dz = dzi + v_s(3);
  double phi = -phi0 - v_s(4);

  transform.setOrigin( tf::Vector3(-(cos(phi)*dx + sin(phi)*dz),0,-(-sin(phi)*dx + cos(phi)*dz)) );
  tf::Quaternion q;
  q.setRPY(0,phi,-beta.data*PI/180);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform,ros::Time::now(),"laser_mount_link","stair_middle"));
  ROS_INFO_ONCE("Transform to stair sent");
}

void Angle::setParameters() {
  n.param("/scalaser/fov_s",fov_s,200);
  n.param("/scalaser/fov_d",fov_d,150);
  n.param("/scalaser/dzi",dzi,.65);
  n.param("/scalaser/phi",phi0,-43.0);
  n.param("/scalaser/kp",kp,0.05);
  n.param("/scalaser/vel_fwd",vel_fwd,0.0);
  n.param("/scalaser/threshold",threshold,0.08);

  phi0 = phi0*PI/180;

  ROS_INFO("Phi0: %f",phi0);
  ROS_INFO("Field of View Start: %d",fov_s);
  ROS_INFO("Field of View Window: %d",fov_d);
  ROS_INFO("Dz Initial: %f",dzi);

  ROS_INFO("Parameters from Server have been updated.");
}

void Angle::plotData() {
  plot_engine.executeCommand("clf");
  plotData(beta_vector);
  plotData(alpha_vector);
  // plotData(dx_1_vector);
  // plotData(dx_2_vector);

  plot_engine.executeCommand("xlabel('Time [s]')");
  plot_engine.executeCommand("legend('beta [°]', 'alpha [°]', 'step diagonal delta [m]')");
}

void Angle::plotData(std::vector<double> data_vector) {
  Eigen::VectorXd data_vectorXd(data_vector.size());
  Eigen::VectorXd time_vectorXd(time_vector.size());

  for (int i=0; i < data_vector.size(); ++i) data_vectorXd[i] = data_vector[i];
  for (int i=0; i < time_vector.size(); ++i) time_vectorXd[i] = time_vector[i];

  plot_engine.put("data_vector", data_vectorXd);
  plot_engine.put("time_vector",time_vectorXd);

  plot_engine.executeCommand("plot_data(time_vector, data_vector)");


  ROS_INFO("Results have been plotted.");

  data_vector.clear();

  // To save a std::vector into a .txt file use:
    // std::ofstream file_beta;
    // file_beta.open("/home/miro/Desktop/cpp2matlab/beta_vector.txt");
    // for(std::size_t i = 0; i < beta_vector.size(); ++i) file_beta << beta_vector[i] << std::endl;
    // file_beta.close();

    // std::ofstream file_time;
    // file_time.open("/home/miro/Desktop/cpp2matlab/time_vector.txt");
    // for(std::size_t i = 0; i < time_vector.size(); ++i) file_time << time_vector[i] << std::endl;
    // file_time.close();
}

void Angle::initializeEdge() {
  edge_marker.header.frame_id = "laser_mount_link";
  edge_marker.ns = "edge";
  edge_marker.id = 1;
  edge_marker.type = visualization_msgs::Marker::ARROW;
  edge_marker.action = 0;
  //edge_marker.pose.orientation.w = 1.0;
  edge_marker.scale.x = 0.05;
  edge_marker.scale.y = 0.05;
  edge_marker.scale.z = 0.001;
  edge_marker.color.a = 1.0; // Don't forget to set the alpha!
  edge_marker.color.r = 0.0;
  edge_marker.color.g = 1.0;
  edge_marker.color.b = 0.0;
}

void Angle::pubEdge() {
  edge_marker.header.stamp = ros::Time::now();
  // Publish Edge around which the wheelchair orients itf
  geometry_msgs::Point p;

  p.x = -( cos(- phi0 - v_r_1(4))*-v_r_1(2) + sin(- phi0 - v_r_1(4))*(dzi + v_r_1(3)));
  p.y = -a/2;
  p.z = -(-sin(- phi0 - v_r_1(4))*-v_r_1(2) + cos(- phi0 - v_r_1(4))*(dzi + v_r_1(3)));
  edge_marker.points.push_back(p);

  p.x = -( cos(- phi0 - v_r_2(4))*-v_r_2(2) + sin(- phi0 - v_r_2(4))*(dzi + v_r_2(3)));
  p.y = a/2;
  p.z = -(-sin(- phi0 - v_r_2(4))*-v_r_2(2) + cos(- phi0 - v_r_2(4))*(dzi + v_r_2(3)));
  edge_marker.points.push_back(p);

  pub_edge_marker.publish(edge_marker);
  edge_marker.points.clear();
}