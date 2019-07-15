#include "my_imu.h"


// using Pose = Eigen::Matrix4f;
My_Imu::My_Imu(const Pose& tf_pose)
  : last_pose_(Pose::Identity())
  , tf_pose_(tf_pose)
{}

My_Imu::~My_Imu() {}

void My_Imu::Initialise( ros::NodeHandle& nh, std::string topic_name )
{
  subscriber_ = nh.subscribe(topic_name, 50, &My_Imu::Callback, this );
  imuMsgs_count = 0;
  newest_time_stamp_ = 0.0;
}

void My_Imu::Callback(const sensor_msgs::Imu::ConstPtr& msg)
{
    if( msg->header.stamp.toSec() <= newest_time_stamp_ ) {
        PRINT_ERROR("Wrong time in imu msg.");
        return;
    }

    double timestamp = msg->header.stamp.toSec() - 1560860000.0;
    timestamp  = int(timestamp * 1000) / 1000.0;
    // ROS_INFO("In imu, time stamp value is: %f", timestamp);

    Eigen::Quaternionf Qwb;
    Qwb = Eigen::Quaternionf(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
    Eigen::Matrix3f Rwb = Qwb.toRotationMatrix();


    LocalImuMsg newImu = l2l_calib::sensors::ToLocalImu(*msg);

    {
        l2l_calib::common::MutexLocker locker(&mutex_);
        newest_time_stamp_ = msg->header.stamp.toSec();
        
        if ( imuMsgs_count == 0 ){
            Rb1w = Rwb.transpose();
        }
        else{
            imu_msgs_.push(newImu);
            vec_mat3f.push_back(Rwb);
            vec_imu_stamp.push_back(timestamp);
        }
    }

    imuMsgs_count++;
}

Eigen::Matrix3f My_Imu::GETimu_rotMat(double lidar_timestamp){
    int stamp_index = 0;
    std::vector<double> vec_stamp = vec_imu_stamp;
    int size_vec_stamp = vec_stamp.size();
    for(int i = 0; i < size_vec_stamp; i++){
        if( abs(vec_stamp[i] - lidar_timestamp) < 0.01 ){
            stamp_index = i;
            break;
        }
        if ( i == size_vec_stamp - 1 ){
            std::cout << "ERROR!" << std::endl << "Fail to find the correspondece time stamp!" << std::endl;
            return Eigen::Matrix3f::Identity();
        }
    }

    Eigen::Matrix3f result = vec_mat3f[stamp_index];
    {
        l2l_calib::common::MutexLocker locker(&mutex_);

        vec_mat3f.erase( vec_mat3f.begin(), vec_mat3f.begin()+stamp_index+1 );
        vec_imu_stamp.erase( vec_imu_stamp.begin(), vec_imu_stamp.begin()+stamp_index+1 );
    }
    return result;
}