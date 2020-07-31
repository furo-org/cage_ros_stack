// Copyright 2018-2020
//       Yoshitaka Hara <hara@furo.org>
//       Tomoaki Yoshida <yoshida@furo.org>
/*
This software is released under the MIT License.
http://opensource.org/licenses/mit-license.php
*/

#include "cageclient.hh"
#include <boost/function.hpp>
#include <boost/program_options.hpp>
#include <boost/program_options/variables_map.hpp>
#include <chrono>
#include <cmath>
#include <limits>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>

namespace bo = boost::program_options;
template <typename Type>
bool parseOption(bo::variables_map &vm, std::string option, Type& store){
  if(vm.count(option)){
    store=vm[option].as<Type>();
    return true;
  }
  return false;
}

// Publisher
class RosBridge{
  ros::NodeHandle Node;
  tf2_ros::TransformBroadcaster TFb;
  struct PublisherDesc
  {
    ros::Publisher pub;
    std::string frame_id;
    std::string child_frame_id;
  };
  std::vector<PublisherDesc> Publishers;
  int ImuId;
public:
  RosBridge();
  ros::NodeHandle& getNodeHandle(){return Node;}


  template <typename msgType>
  int NewPublisher(std::string topic, uint32_t qlen, std::string frame_id, std::string child_frame_id);

  void PublishOdom(int navid, ros::Time stamp, double x, double y, double z, tf2::Quaternion q, double vx, double az);
  void PublishIMU(ros::Time stamp, double ow, double ox, double oy, double oz, double rx, double ry, double rz, double ax, double ay, double az);
  void PublishTFTransform(ros::Time stamp, std::string frame_id, std::string child_frame_id, double x, double y, double z, tf2::Quaternion q);

private:
 nav_msgs::Odometry BuildNavMsg(double x, double y, double z, tf2::Quaternion q, double vx, double az);
};

// Odometry calculater
class SimpleOdometry{
  const CageAPI::vehicleInfo Info;
  double LastClock;

public:
  double X = 0, Y = 0, Th = 0, Vx = 0, Az = 0;

public:
  SimpleOdometry(const CageAPI::vehicleInfo &info) : Info(info), LastClock(std::numeric_limits<double>::max()){};
  bool Accumulate(const CageAPI::vehicleStatus &st);
  void Reset(double x=0, double y=0, double th=0){X=x;Y=y;Th=th;}
};

// ---------

RosBridge::RosBridge(){
  ImuId = NewPublisher<sensor_msgs::Imu>("imu",100,"base_link","imu_link");
}

template<typename msgType>
int RosBridge::NewPublisher(std::string topic, uint32_t qlen, std::string frame_id, std::string child_frame_id)
{
  Publishers.push_back(PublisherDesc{Node.advertise<msgType>(topic, qlen),frame_id, child_frame_id});
  return Publishers.size()-1;
}

nav_msgs::Odometry RosBridge::BuildNavMsg(double x, double y, double z, tf2::Quaternion q, double vx, double az)
{
  nav_msgs::Odometry odo;
  odo.pose.pose.position.x = x;
  odo.pose.pose.position.y = y;
  odo.pose.pose.position.z = z;
  odo.pose.pose.orientation = tf2::toMsg(q);
  odo.pose.covariance = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  odo.twist.twist.linear.x = vx;
  odo.twist.twist.linear.y = 0.;
  odo.twist.twist.linear.z = 0.;
  odo.twist.twist.angular.x = 0.;
  odo.twist.twist.angular.y = 0.;
  odo.twist.twist.angular.z = az;
  odo.twist.covariance = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                          0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                          0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  return odo;
}

void RosBridge::PublishOdom(int pubid, ros::Time stamp, double x, double y, double z, tf2::Quaternion q, double vx, double az)
{
  if(pubid>=Publishers.size())return;
  const PublisherDesc &np=Publishers[pubid];

  // publish
  nav_msgs::Odometry odo=BuildNavMsg(x,y,z,q ,vx,az);
  odo.header.frame_id = np.frame_id;
  odo.header.stamp = stamp;
  odo.child_frame_id = np.child_frame_id;

  np.pub.publish(odo);
  PublishTFTransform(stamp,np.frame_id, np.child_frame_id, x,y,z,q);
}

void RosBridge::PublishIMU(ros::Time stamp, double ow, double ox, double oy, double oz, double rx, double ry, double rz, double ax, double ay, double az){
  const PublisherDesc &np = Publishers[ImuId];

  sensor_msgs::Imu imu;
  imu.header.frame_id = np.frame_id;
  imu.header.stamp=stamp;
  imu.orientation.w = ow;
  imu.orientation.x = ox;
  imu.orientation.y = oy;
  imu.orientation.z = oz;
  imu.orientation_covariance = {0, 0, 0, 0, 0, 0, 0, 0, 0};
  imu.angular_velocity.x = rx;
  imu.angular_velocity.y = ry;
  imu.angular_velocity.z = rz;
  imu.angular_velocity_covariance = {0, 0, 0, 0, 0, 0, 0, 0, 0};
  imu.linear_acceleration.x = ax;
  imu.linear_acceleration.y = ay;
  imu.linear_acceleration.z = az;
  imu.linear_acceleration_covariance = {0, 0, 0, 0, 0, 0, 0, 0, 0};
  np.pub.publish(imu);

  tf2::Quaternion q;
  q.setRPY(0,0,0);
  PublishTFTransform(stamp,np.frame_id, np.child_frame_id, 0,0,0,q);
}

void RosBridge::PublishTFTransform(ros::Time stamp, std::string frame_id, std::string child_frame_id, double x, double y, double z, tf2::Quaternion q){
  geometry_msgs::TransformStamped tf_dev;
  tf_dev.header.stamp = stamp;
  tf_dev.header.frame_id = frame_id;
  tf_dev.child_frame_id = child_frame_id;
  tf_dev.transform.translation.x = x;
  tf_dev.transform.translation.y = y;
  tf_dev.transform.translation.z = z;
  tf_dev.transform.rotation = tf2::toMsg(q);
  TFb.sendTransform(tf_dev);
}

// ---------

bool SimpleOdometry::Accumulate(const CageAPI::vehicleStatus &st){
  if(LastClock>st.simClock){
    // reset odometry
    LastClock=st.simClock;
    X=0;
    Y=0;
    Th=0;
    return false;
  }
  // simple odometry
  double dt = st.simClock - LastClock;
  double ts;

  if (dt < 0.001) //1ms
    return false;
  LastClock = st.simClock;

  double vr = -st.rrpm * Info.WheelPerimeterR / Info.ReductionRatio / 60.;
  double vl = st.lrpm * Info.WheelPerimeterL / Info.ReductionRatio / 60.;
  double vx = (vr + vl) / 2.;
  //az=(vr-vl)/treadWidth; // use wheel rotation and body parameters
  double az = st.rz; // use angular velocity sensor
  double dx = vx * cos(Th) * dt;
  double dy = vx * sin(Th) * dt;
  X += dx;
  Y += dy;
  Th += az * dt;
  Vx=vx;
  Az=az;
  return true;
}

int main(int argc, char **argv) try
{
    bo::options_description opts("Options");
    opts.add_options()("help,h", "Show usage")("version,v", "Show program version");
    opts.add_options()("device,d", bo::value<std::string>());

    bo::variables_map vm;
    std::string device, port;
    std::vector<std::string> command;
    bool scan = false;
    int canspeed = 0;
    std::cout << "CageRos" << std::endl;
    std::cout << "   2019/02/20  yoshida@furo.org" << std::endl;

    try
    {
        bo::store(bo::parse_command_line(argc, argv, opts), vm);
        if (vm.count("help"))
        {
            std::cout << opts << std::endl;
            exit(0);
        }
        if (vm.count("version"))
        {
            exit(0);
        }
        if (!parseOption(vm, "device", device))
        {
            std::cerr << "No device specified." << std::endl;
            exit(0);
        }
    }
    catch (std::exception &e)
    {
        std::cerr << e.what() << std::endl;
        std::cout << opts << std::endl;
        exit(0);
    }

    ros::init(argc, argv, "cage_ros");

    CageAPI Cage(device);
    if(!Cage.connect()){
        std::cerr<<"Failed to connect:"<<Cage.getError()<<std::endl;
        return -1;
    }

    RosBridge rosIF;

    int pubOdom = rosIF.NewPublisher<nav_msgs::Odometry>("odom", 100, "odom", "base_link");
    int pubOdom_gt = rosIF.NewPublisher<nav_msgs::Odometry>("odom_gt", 100, "odom", "base_link_gt");

    boost::function<void(const geometry_msgs::Twist &r_msg)>
        fromRos =
            [&Cage](const auto &msg) {
                std::cout << "setVW:"<< msg.linear.x <<"," << msg.angular.z << std::endl;
                Cage.setVW(msg.linear.x, msg.angular.z);
            };
    auto sub = rosIF.getNodeHandle().subscribe<geometry_msgs::Twist>("cmd_vel", 10, fromRos);

    ros::AsyncSpinner spinner(1);
    spinner.start();

    CageAPI::vehicleStatus st;
    if (!Cage.getStatusOne(st, 100)) {
      std::cerr << "getStatus failed:" << Cage.getError() << std::endl;
      exit(0);
    }

    SimpleOdometry odo(Cage.VehicleInfo);
    odo.Accumulate(st);

    std::cout << "Vehicle Parameters\n tread, wheelPerimeterR,L reductionRatio"
              << Cage.VehicleInfo.TreadWidth << " " << Cage.VehicleInfo.WheelPerimeterR
              << " " << Cage.VehicleInfo.WheelPerimeterL << " " << Cage.VehicleInfo.ReductionRatio << std::endl;

    while (ros::ok())
    {
      if (!Cage.getStatusOne(st, 100)) {
        std::cerr << "getStatus failed:" << Cage.getError() << std::endl;
        continue;
      }

      // to ROS
      auto stamp=ros::Time::now();

      if(!odo.Accumulate(st)){
        // world was restarted
        Cage.setVW(0,0);
        continue;
      }

#if 0
      //std::cout<<"rrpm, lrpm, vr, vl, dt"<<st.rrpm<<" "<<st.lrpm<<" "<<vr<<"  "<<vl<<"  "<<dt<<std::endl;
      std::cout << "rx, ry, rz " << std::setw(12) << std::fixed << st.rz << " "
                << std::setw(12) << std::fixed << st.ry << " " << std::setw(12)
                << std::fixed << st.rz << std::endl;
#endif

      // odometry
      tf2::Quaternion q;
      q.setRPY(0, 0, odo.Th);
      rosIF.PublishOdom(pubOdom, stamp, odo.X, odo.Y, 0, q, odo.Vx, odo.Az);

      // ground truth
      tf2::Quaternion qgt(st.ox,st.oy,st.oz,st.ow);
      rosIF.PublishOdom(pubOdom_gt, stamp, st.wx, st.wy, st.wz, qgt, odo.Vx, st.rz);

      // Orientation: Ground Truth,  Angulrar velocity, Linear Acceleration: values from physics engine
      rosIF.PublishIMU(stamp, st.ow, st.ox, st.oy, st.oz, st.rx, st.ry, st.rz, st.ax, st.ay, st.az);

      // Scanner position
      q.setRPY(0, 0, 0);
      rosIF.PublishTFTransform(stamp, "base_link", "lidar3d_link", -.22, 0, .518 , q);
    }
    spinner.stop();
    return 0;
}
catch (const std::exception &ex)
{
    std::cerr << "EXCEPTION: " << ex.what() << std::endl;
    return 1;
}
