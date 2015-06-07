#include "matching.h"

matlab::Engine matching::engine(true);  // Initialize static matlab engine

matching::matching(ros::NodeHandle n_, double phi0_, double dzi_, int fov_s_, int fov_d_, Eigen::VectorXd v0_, int h_):
n(n_),
phi0(phi0_), dzi(dzi_), fov_s(fov_s_), fov_d(fov_d_), h(h_),
xi(fov_d_), zi(fov_d_), xf(fov_d_), zf(fov_d), z_r(fov_d_),
v0(v0_),
v_r(v0_.size()), lb(v0_.size()), ub(v0_.size()),
se_r(0),
ang_inc(0.00581718236208), min_ang(-2.35619449019), max_ang(2.35619449019)
{
  // engine.showWorkspace();
  n.param("/scalaser/threshold", threshold, 0.08);
  if (engine.good()) {ROS_INFO("Engine succesfully initialized.");}
  // Set path to matlab directory
  engine.changeWorkingDirectory("~/catkin_ws/src/scalaser/matlab");

  // Initialize upper and lower bound vector for fminsearch
  lb << 0.13, 0.21, -0.4, -10, -PI;
  ub << 0.20, 0.50, 0.4, 10, PI;
}


void matching::matchCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
  r_temp.clear();
  for (int i = 0; i < 811; i++) {
    r_temp.push_back(msg->ranges[i]);
  }
}

void matching::setData() {
  int rs_it = 0;  // Resize Itterator

  xi_temp.clear();
  zi_temp.clear();
  
  for (int i = fov_s; i < fov_s + fov_d + rs_it; i++) {
    if (r_temp[i] < 0.3) {
      rs_it++;
    }
    else {
    xi_temp.push_back(cos(i*ang_inc + min_ang)*r_temp[i]);
    zi_temp.push_back(sin(i*ang_inc + min_ang)*r_temp[i]);
    }
  }
  if(rs_it > 0) {ROS_INFO("%d INVALID values within FoV of Sensor %d.", rs_it, h);}
}

void matching::transformMsg() {

  // Transform Vector around initial guess
  for (int i=0; i < fov_d; i++) {
    xi(i) = (- zi_temp[i]*cos(phi0)*pow((-1), (h-1)) + xi_temp[i]*sin(phi0));
    zi(i) = (- xi_temp[i]*cos(phi0) - zi_temp[i]*sin(phi0)*pow((-1), (h-1)) + dzi);
  }
}

void matching::matchTemplate() {
  transformMsg();
  fillMatfile();
  // fillEngine();
  publishSe_r();
}

void matching::publishSe_r() {
    std_msgs::Float64 se_rM;
    se_rM.data = se_r;
    std::string topic = "/se_r_";
    topic += std::to_string(h);
    se_r_pub = n.advertise<std_msgs::Float64>(topic, 1000);
    se_r_pub.publish(se_rM);
}

void matching::setFminArgs(Eigen::VectorXd v_r_) {
  v0 = v_r_;
  lb(2) = v0(2) - sqrt(v0(1)*v0(1) + v0(0)*v0(0))/2;
  ub(2) = v0(2) + sqrt(v0(1)*v0(1) + v0(0)*v0(0))/2;
  // ROS_INFO("Lower boundary: %f",lb(2));
  // ROS_INFO("Upper boundary: %f",ub(2));
}

void matching::setParameters(double phi0_,double dzi_,int fov_s_,int fov_d_) {
  phi0 = phi0_;
  dzi = dzi_;
  fov_s = fov_s_;
  fov_d = fov_d_;

  xi.resize(fov_d);
  zi.resize(fov_d);
  xf.resize(fov_d);
  zf.resize(fov_d);
  z_r.resize(fov_d);
}

void matching::fillMatfile() {
  file.open("~/catkin_ws/src/scalaser/matlab/stairparam.mat", matlab::MatFile::WRITE_COMPRESSED);
  assert(file.isOpen());
  assert(file.isWritable());

  file.put("xi", xi);
  file.put("zi", zi);
  file.put("h", h);
  file.put("lb", lb);
  file.put("ub", ub);
  file.put("v0", v0);

  engine.executeCommand("load('stairparam.mat')");

  // engine.put("xi",xi);
  assert(file.close());

  // ROS_INFO("BEFORE FMINSEARCH()");
  // ROS_INFO("stair heigth________h = %f",v_r(0)); 
  // ROS_INFO("stair depth_________t = %f",v_r(1));
  // ROS_INFO("phase offset_______dx = %f",v_r(2));
  // ROS_INFO("sensor height______dz = %f",v_r(3));
  // ROS_INFO("sensor rotation___phi0 = %f",v_r(4));
  // ROS_INFO("__________________________________");

  engine.executeCommand("test(xi,zi,h)");

  double before = ros::Time::now().toSec();
  engine.executeCommand("[v_r, se_r, z_r, xf, zf] = stairparam(xi, zi, v0, h, lb, ub);");
  double after = ros::Time::now().toSec();
  // ROS_INFO("Time to compute fmincon %f", after - before);

  engine.executeCommand("save('stairparam.mat')");

  file.open("~/catkin_ws/src/scalaser/matlab/stairparam.mat", matlab::MatFile::READ);
  assert(file.isOpen());
  assert(!file.isWritable());

  file.get("se_r", se_r);
  file.get("v_r", v_r);
  file.get("z_r", z_r);
  file.get("xf", xf);
  file.get("zf", zf);

  assert(file.close());

  // ROS_INFO("RESULT VECTOR AFTER FMINSERACH() OF CLOUD_%d",h);
  // ROS_INFO("stair heigth________h = %f",v_r(0)); 
  // ROS_INFO("stair depth_________t = %f",v_r(1));
  // ROS_INFO("phase offset_______dx = %f",v_r(2));
  // ROS_INFO("sensor height______dz = %f",v_r(3));
  // ROS_INFO("sensor rotation___phi0 = %f",v_r(4));


  // ROS_INFO("Result of fminsearch(): %f",se_r);
  // ROS_INFO("__________________________________");
  // ROS_INFO("__________________________________");


  // Check if solution satisfies threshold

  if (se_r < threshold) {
    setFminArgs(v_r);
  }
  else {
    ROS_WARN("Pointcloud has not been matched properly. Error: %f", se_r);
  }
}

void matching::fillEngine() {
  //  ROS_INFO("Matching Template of Cloud_%d",h);
  engine.put("xi", xi);
  engine.put("zi", zi);
  engine.put("h", h);
  engine.put("lb", lb);
  engine.put("ub", ub);

  //  engine.executeCommand("test(xi,zi,h)");
  if (se_r < 0.008) v0 = v_r;
  engine.put("v0", v0);

  engine.executeCommand("[v_r, se_r, z_r, xf, zf] = stairparam(xi, zi, v0, h, lb, ub);");
  engine.get("se_r", se_r);
  engine.get("v_r", v_r);

  engine.get("z_r", z_r);
  engine.get("xf", xf);
  engine.get("zf", zf);

  if (se_r < threshold) {
    setFminArgs(v_r);
  }
  else {
    ROS_WARN("Pointcloud has not been matched properly. Error: %f", se_r);
  }

}