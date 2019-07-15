#include "imu.h"
#include "msg_conversion.h"

namespace l2l_calib {

// using Pose = Eigen::Matrix4f;
Imu::Imu(const Pose& tf_pose)
  : last_pose_(Pose::Identity())
  , tf_pose_(tf_pose)
  , bias_(gtsam::Vector3(0,0,0), gtsam::Vector3(0,0,0))
  , imu_current_estimate_(bias_,
      gtsam::Matrix3::Zero(), gtsam::Matrix3::Zero(), gtsam::Matrix3::Zero(),
      gtsam::Matrix3::Zero(), gtsam::Matrix3::Zero(), gtsam::Matrix::Zero(6,6))
{}

Imu::~Imu() {}

void Imu::Initialise( ros::NodeHandle& nh, std::string topic_name )
{
  subscriber_ = nh.subscribe(topic_name, 50, &Imu::Callback, this );
  imuMsgs_count = 0;
}

void Imu::Callback(const sensor_msgs::Imu::ConstPtr& msg)
{
    if( conversion::ToLocalTime(msg->header.stamp) <= newest_time_stamp_ ) {
      PRINT_ERROR("Wrong time in imu msg.");
      return;
    }

    double timestamp = msg->header.stamp.toSec();
    ROS_INFO("In imu, time stamp value is: %f", timestamp);
    // if (imuMsgs_count % 5 == 0)
    // {
      // remove the gravity
    double roll, pitch, yaw;
    tf::Quaternion orientation;
    tf::quaternionMsgToTF(msg->orientation, orientation);
    tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

    // 对imuShiftX, imuVeloX的计算有影响
    double accX = msg->linear_acceleration.x + sin(pitch) * 9.94;
    double accY = msg->linear_acceleration.y - sin(roll) * cos(pitch) * 9.94;
    double accZ = msg->linear_acceleration.z - cos(roll) * cos(pitch) * 9.94;

    LocalImuMsg newImu = sensors::ToLocalImu(*msg);
    newImu.linear_acceleration.x = accX;
    newImu.linear_acceleration.y = accY;
    newImu.linear_acceleration.z = accZ;

    {
      common::MutexLocker locker(&mutex_);
      newest_time_stamp_ = conversion::ToLocalTime(msg->header.stamp);
      imu_msgs_.push(newImu);

      if (imuMsgs_count % 20 == 0)
        queue_targetTime.push(newest_time_stamp_);
    }
    // std::cout << "imu : " << std::endl;
      // std::cout << "imu sec : " << sensors::ToLocalImu(*msg).header.stamp.secs << std::endl;
      // std::cout << "imu nsec : " << sensors::ToLocalImu(*msg).header.stamp.nsecs << std::endl;
      // std::cout << "angular velocity : " << sensors::ToLocalImu(*msg).angular_velocity.x << " , "
      //   << sensors::ToLocalImu(*msg).angular_velocity.y << " , "
      //   << sensors::ToLocalImu(*msg).angular_velocity.z << std::endl;
      // std::cout << "linear_accleration : " << newImu.linear_acceleration.x << " , "
      //   << newImu.linear_acceleration.y << " , "
      //   << newImu.linear_acceleration.z << std::endl;
      // std::cout << "quaternion : " << sensors::ToLocalImu(*msg).orientation.x << " , "
      //   << sensors::ToLocalImu(*msg).orientation.y << " , "
      //   << sensors::ToLocalImu(*msg).orientation.z << " , "
      //   << sensors::ToLocalImu(*msg).orientation.w << std::endl;
    // }
    imuMsgs_count++;
}

bool Imu::GetIntergratedPose( const SimpleTime& target_time,
  Pose& pose, Velocity3& velocity )
{
  if( target_time > newest_time_stamp_ ) {
    PRINT_WARNING("please wait for new imu data.");
    pose = last_pose_;
    return false;
  }

  std::cout << "queue imu_msgs_ size : " << imu_msgs_.size() << std::endl;
  std::vector<LocalImuMsg> used_imus;
  while( imu_msgs_.front().header.stamp <= target_time ) {
    used_imus.push_back(imu_msgs_.front()); // std::queue<LocalImuMsg> imu_msgs_
    imu_msgs_.pop();
  }
  std::cout << "vector used_imus size : " << used_imus.size() << std::endl;

  for( auto& imu : used_imus ) {
    gtsam::Vector3 acc(imu.linear_acceleration.x, 
      imu.linear_acceleration.y, imu.linear_acceleration.z);
    gtsam::Vector3 angle_velocity(imu.angular_velocity.x, 
      imu.angular_velocity.y, imu.angular_velocity.z);
    imu_current_estimate_.integrateMeasurement( acc, angle_velocity, 0.025);
  }
  gtsam::Rot3 last_rotation = imu_current_estimate_.deltaRij();
  gtsam::Vector3 last_position = imu_current_estimate_.deltaPij();
  gtsam::Vector3 last_velocity = imu_current_estimate_.deltaVij();
  velocity = last_velocity.cast<float>();
  
  Pose target_time_pose = Pose::Identity();
  target_time_pose.block(0,3,3,1) = last_position.cast<float>();
  target_time_pose.block(0,0,3,3) = last_rotation.matrix().cast<float>();

  pose = last_pose_ = target_time_pose;
  return true;
}

} // namespace l2l_calib