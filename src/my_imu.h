#ifndef MY_IMU_H_
#define MY_IMU_H_

#include "common.h"
#include "sensors.h"
#include "ros/ros.h"
#include <queue>

#include <tf_conversions/tf_eigen.h>
// #include <Eigen/Core>
// #include <Eigen/Geometry>


class My_Imu {
public:
  using LocalImuMsg = l2l_calib::sensors::ImuMsg;
  using Pose = Eigen::Matrix4f;
  using Velocity3 = Eigen::Vector3f;

  My_Imu(const Pose& tf_pose);
  ~My_Imu();

  /// @brief init the ros topic and subscriber
  void Initialise( ros::NodeHandle& nh, std::string topic_name );
  /*
   * @brief return a pre-intergrated pose on target time
   * @param time : target time 
   * @param pose: output the pose on target time
   * @param velocity: output the velocity on target time
   * @return true if succeed otherwise false
  */
  bool GetIntergratedPose( const SimpleTime& time, 
    Pose& pose, Velocity3& velocity );

  inline 
  Pose GetTfPose() { return tf_pose_; }

  inline
  int GETImuMsgs_count(){ return imuMsgs_count; }

  inline
  void SETLast_Pose(){ last_pose_ = Pose::Identity(); }

  inline
  Pose GETLast_Pose(){ return last_pose_; }

  inline
  std::vector<Eigen::Matrix3f> GETVec_Mat3f() {return vec_mat3f;}

  inline
  Eigen::Matrix3f GETRb1w() { return Rb1w; }

  Eigen::Matrix3f GETimu_rotMat(double lidar_timestamp);

protected:
  void Callback(const sensor_msgs::Imu::ConstPtr& msg);

private:
  l2l_calib::common::Mutex mutex_;

  std::queue<LocalImuMsg> imu_msgs_;
  std::vector<Eigen::Matrix3f> vec_mat3f;   // rotation mats of imu data flow
  Eigen::Matrix3f Rb1w;   // the first frame of the imu data flow

  ros::Subscriber subscriber_;

  int imuMsgs_count;
  double newest_time_stamp_;
  std::vector<double> vec_imu_stamp;

  Pose last_pose_;
  Pose tf_pose_;

};

#endif