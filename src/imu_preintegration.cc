#include <iostream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "calibrator.h"
#include "pcl_conversions/pcl_conversions.h"
#include <pcl/console/parse.h>
#include "options.h"
#include "common/math.h"
// #include <tf_conversions/tf_eigen.h>
#include <tf/transform_broadcaster.h>
// #include <tf/transform_listener.h>

using namespace l2l_calib;
using namespace l2l_calib::common;

std::shared_ptr<Imu> imu_;
struct ImuMotionData {
    Imu::Pose pose;
    Imu::Velocity3 velocity;
  };
std::vector<ImuMotionData> motion_data;


void publishTF(Imu::Pose imuPose, tf::StampedTransform& imuTrans)
{
    Eigen::Quaterniond eigen_q(imuPose.block<3,3>(0,0).cast<double>());
    Eigen::Vector3d eigen_T(imuPose.block<3,1>(0,3).cast<double>());
    imuTrans.setRotation(tf::Quaternion(eigen_q.x(), eigen_q.y(), eigen_q.z(), eigen_q.w()));
    imuTrans.setOrigin(tf::Vector3(eigen_T(0),eigen_T(1),eigen_T(2)));
    imuTrans.stamp_ = ros::Time::now();
}

int main(int argc, char **argv)
{ 
  ros::init(argc, argv, "imu_preintegration");
  ros::NodeHandle n;

  tf::TransformBroadcaster tfBroadcaster;
  tf::StampedTransform imuOdometryTrans;
  imuOdometryTrans.frame_id_ = "imu_init";
  imuOdometryTrans.child_frame_id_ = "imu";

  // *********从gem.xml中读取lidar和imu的配置参数*********
  // *****配置参数包括topic name, translation, quaternion*****
  // *****lidar和imu的配置参数首先一起存放在options中，然后分别转存入settings.lidar_settings和settings.imu_settings****
  std::string cfg_filename = "";
  pcl::console::parse_argument(argc, argv, "-cfg", cfg_filename);
  if( cfg_filename.empty() ) {
    std::cout << "You should use the node this way:\n\n"
      << "    l2l_calib_node -cfg [config_file_name.xml]\n" << std::endl;
    return -1;
  }

  std::vector<l2l_calib::DeviceCoordOption> options;
  if( l2l_calib::ReadOptions(cfg_filename, options) < 0 || options.empty()) {
    return -1;
  }

  l2l_calib::CalibratorSettings settings;
  auto& lidar_settings = settings.lidar_settings;   //std::vector<LidarSetting> lidar_settings;
  auto& imu_settings = settings.imu_settings;
  for( auto& option : options ) {
    l2l_calib::TfSetting setting;
    setting.topic_name = option.topic_name;
    setting.translation << option.translation.x,
      option.translation.y, option.translation.z;
    auto& q = option.quaternion;
    setting.quaternion = Eigen::Quaternionf(
      q.w, q.x, q.y, q.z );

    switch(option.type) {
      case l2l_calib::kLidar:
        lidar_settings.push_back(setting);
        break;
      case l2l_calib::kImu:
        imu_settings.push_back(setting);
        break;
      default:
        break;
    }
  }
  // *********从gem.xml中读取lidar和imu的配置参数*********

  if( !imu_settings.empty() ) {
    auto imus_count = imu_settings.size();
    //imus_.resize(imus_count);
    // imus_motion_data_.resize(imus_count);
    std::cout << "In calibrator.cc, imus_count : " << imus_count << std::endl;

    Eigen::Matrix4f tf_pose = Eigen::Matrix4f::Identity();
    tf_pose.block(0,0,3,3) = 
      Eigen::Matrix3f( imu_settings[0].quaternion.toRotationMatrix() );
    tf_pose.block(0,3,3,1) = imu_settings[0].translation;

    imu_ = std::make_shared<Imu>( tf_pose );
    imu_->Initialise( n, imu_settings[0].topic_name );  //初始化之后便开始订阅imu数据
  }

  ros::Rate loop_rate(10);
  while( ros::ok() )
  {
    ros::spinOnce();

    if (imu_->GETImuMsgs_count() <= 200)
      continue;

    ImuMotionData preintegrate_result;
    Imu::Pose pose;
    Imu::Velocity3 vel;
    SimpleTime target_time = imu_->queue_targetTime.front();
    imu_->queue_targetTime.pop();
    if( imu_->GetIntergratedPose(target_time, pose, vel) ) {
      preintegrate_result.pose = pose;
      preintegrate_result.velocity = vel;
      motion_data.push_back(preintegrate_result);
      std::cout << "preintegration pose :" << std::endl;
      std::cout << preintegrate_result.pose << std::endl;
      std::cout << "preintegration velocity :" << std::endl;
      std::cout << preintegrate_result.velocity << std::endl;
      publishTF(pose, imuOdometryTrans);
      tfBroadcaster.sendTransform(imuOdometryTrans);
      imu_->SETLast_Pose();
      std::cout << "reset last_pose_ " << std::endl;
      std::cout << imu_->GETLast_Pose() << std::endl;
    }
    
    loop_rate.sleep();
  }
  
  return 0;
}


