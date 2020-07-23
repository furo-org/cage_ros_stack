
#include "cageclient.hh"
#include <boost/function.hpp>
#include <boost/program_options.hpp>
#include <boost/program_options/variables_map.hpp>
#include <chrono>
#include <cmath>
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

    ros::NodeHandle nh;
    ros::Publisher pub_odom = nh.advertise<nav_msgs::Odometry>("odom", 100);
    ros::Publisher pub_odomgt = nh.advertise<nav_msgs::Odometry>("odom_gt", 100);
    ros::Publisher pub_imu = nh.advertise<sensor_msgs::Imu>("imu", 100);
    tf2_ros::TransformBroadcaster tfb;

    tf2::Quaternion iq;
    iq.setRPY(0., 0., 0.);
    tf2::Transform imuTrans(iq, tf2::Vector3{0, 0, 0});

    boost::function<void(const geometry_msgs::Twist &r_msg)>
        fromRos =
            [&Cage](const auto &msg) {
                std::cout << "setVW:"<< msg.linear.x <<"," << msg.angular.z << std::endl;
                Cage.setVW(msg.linear.x, msg.angular.z);
            };
    auto sub = nh.subscribe<geometry_msgs::Twist>("cmd_vel", 10, fromRos);

    ros::AsyncSpinner spinner(1);
    spinner.start();

    double odox=0, odoy=0, odoth=0, vx, av, dt;
    double simClock=0;
    {
      CageAPI::vehicleStatus st;
      if (!Cage.getStatusOne(st, 100)) {
        std::cerr << "getStatus failed:" << Cage.getError() << std::endl;
        exit(0);
      }
      simClock=st.simClock;
      odox=st.wx;
      odoy=st.wy;
      auto q = tf2::Quaternion(st.ox, st.oy, st.oz, st.ow);
      double ir,ip,iy;
      tf2::Matrix3x3(q).getRPY(ir,ip,iy);
      odoth=iy;
    }

    const double treadWidth=Cage.VehicleInfo.TreadWidth;
    const double perimeterR = Cage.VehicleInfo.WheelPerimeterR;
    const double perimeterL = Cage.VehicleInfo.WheelPerimeterL;
    const double reductionRatio=Cage.VehicleInfo.ReductionRatio;

    std::cout<<"Vehicle Parameters\n tread, wheelPerimeterR,L reductionRatio"
    <<treadWidth<<" "<<perimeterR<<" "<<perimeterL<<" "<<reductionRatio<<std::endl;

    while (ros::ok()) {
      CageAPI::vehicleStatus st;
      if (!Cage.getStatusOne(st, 100)) {
        std::cerr << "getStatus failed:" << Cage.getError() << std::endl;
        continue;
      }
      // to ROS
      // simple odometry
      dt=st.simClock-simClock;
      double ts;
      int32_t tsnsec= static_cast <int32_t>(std::modf(st.simClock,&ts)*1e9);
      int32_t tssec=static_cast<int32_t>(ts);

      if(dt<0.001) continue; //1ms
      simClock=st.simClock;

      double vr = -st.rrpm * perimeterR/reductionRatio / 60.;
      double vl = st.lrpm * perimeterL/reductionRatio / 60.;
      vx = (vr+vl)/2.;
      //av=(vr-vl)/treadWidth; // use wheel rotation and body parameters
      av=st.rz;  // use angular velocity sensor
      double dx = vx * cos(odoth)*dt;
      double dy=vx*sin(odoth)*dt;
      odox+=dx;
      odoy+=dy;
      odoth+=av*dt;
      //std::cout<<"rrpm, lrpm, vr, vl, dt"<<st.rrpm<<" "<<st.lrpm<<" "<<vr<<"  "<<vl<<"  "<<dt<<std::endl;
      std::cout << "rx, ry, rz " << std::setw(12) << std::fixed << st.rz << " "
                << std::setw(12) << std::fixed << st.ry << " " << std::setw(12)
                << std::fixed << st.rz << std::endl;
      // publish
      nav_msgs::Odometry odo;
      odo.header.frame_id="odom";
      odo.header.stamp.sec = tssec;
      odo.header.stamp.nsec = tsnsec;
      odo.child_frame_id = "base_link";
      odo.pose.pose.position.x = odox;
      odo.pose.pose.position.y = odoy;
      odo.pose.pose.position.z = 0.;
      tf2::Quaternion q;
      q.setRPY(0., 0., odoth);
      odo.pose.pose.orientation = tf2::toMsg(q);
      odo.pose.covariance = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                             0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                             0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
      odo.twist.twist.linear.x = vx;
      odo.twist.twist.linear.y = 0.;
      odo.twist.twist.linear.z = 0.;
      odo.twist.twist.angular.x = 0.;
      odo.twist.twist.angular.y = 0.;
      odo.twist.twist.angular.z = av;
      odo.twist.covariance = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                              0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                              0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
      pub_odom.publish(odo);
      geometry_msgs::TransformStamped odotf;
      odotf.header.frame_id="odom";
      odotf.header.stamp.sec=tssec;
      odotf.header.stamp.nsec=tsnsec;
      odotf.child_frame_id="base_link";
      odotf.transform.translation.x = odo.pose.pose.position.x;
      odotf.transform.translation.y = odo.pose.pose.position.y;
      odotf.transform.translation.z = odo.pose.pose.position.z;
      odotf.transform.rotation=tf2::toMsg(q);
      tfb.sendTransform(odotf);

      nav_msgs::Odometry odogt;
      odogt.header.frame_id = "odom";
      odogt.header.stamp.sec = tssec;
      odogt.header.stamp.nsec = tsnsec;
      odogt.child_frame_id = "base_link_gt";
      odogt.pose.pose.position.x = st.wx;
      odogt.pose.pose.position.y = st.wy;
      odogt.pose.pose.position.z = st.wz;
      q=tf2::Quaternion(st.ox,st.oy,st.oz,st.ow);
      odogt.pose.pose.orientation = tf2::toMsg(q);
      odogt.pose.covariance = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                             0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                             0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
      odogt.twist.twist.linear.x = vx;
      odogt.twist.twist.linear.y = 0.;
      odogt.twist.twist.linear.z = 0.;
      odogt.twist.twist.angular.x = 0.;
      odogt.twist.twist.angular.y = 0.;
      odogt.twist.twist.angular.z = st.rz;
      odogt.twist.covariance = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                              0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                              0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
      pub_odomgt.publish(odogt);
      geometry_msgs::TransformStamped odogttf;
      odogttf.header.frame_id = "odom";
      odogttf.header.stamp.sec = tssec;
      odogttf.header.stamp.nsec = tsnsec;
      odogttf.child_frame_id = "base_link_gt";
      odogttf.transform.translation.x = odogt.pose.pose.position.x;
      odogttf.transform.translation.y = odogt.pose.pose.position.y;
      odogttf.transform.translation.z = odogt.pose.pose.position.z;
      odogttf.transform.rotation = tf2::toMsg(q);
      tfb.sendTransform(odogttf);

      sensor_msgs::Imu imu;
      imu.header.frame_id = "imu";
      imu.header.stamp.sec = tssec;
      imu.header.stamp.nsec = tsnsec;
      imu.orientation.w = st.ow;
      imu.orientation.x=st.ox;
      imu.orientation.y=st.oy;
      imu.orientation.z=st.oz;
      imu.orientation_covariance = {0, 0, 0, 0, 0, 0, 0, 0, 0};
      imu.angular_velocity.x = st.rx;
      imu.angular_velocity.y = st.ry;
      imu.angular_velocity.z = st.rz;
      imu.angular_velocity_covariance = {0, 0, 0, 0, 0, 0, 0, 0, 0};
      imu.linear_acceleration.x = st.ax;
      imu.linear_acceleration.y = st.ay;
      imu.linear_acceleration.z = st.az;
      imu.linear_acceleration_covariance = {0, 0, 0, 0, 0, 0, 0, 0, 0};
      pub_imu.publish(imu);

      geometry_msgs::TransformStamped tf_imu;
      tf_imu.header.stamp.sec=tssec;
      tf_imu.header.stamp.nsec=tsnsec;
      tf_imu.header.frame_id="base_link";
      tf_imu.child_frame_id="imu";
      tf_imu.transform=tf2::toMsg(imuTrans);
      tfb.sendTransform(tf_imu);
    }
    spinner.stop();
    return 0;
}
catch (const std::exception &ex)
{
    std::cerr << "EXCEPTION: " << ex.what() << std::endl;
    return 1;
}
