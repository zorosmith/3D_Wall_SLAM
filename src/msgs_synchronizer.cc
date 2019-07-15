#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <ros/ros.h>
#include <iostream>

using namespace std;

class mySynchronizer{
public:
    ros::Publisher pubVelodyne;
    ros::Publisher pubImu;

    mySynchronizer();
    ~mySynchronizer(){};

    void callback(const sensor_msgs::PointCloud2::ConstPtr& ori_pointcloud, const sensor_msgs::Imu::ConstPtr& ori_imu);
    
};

void mySynchronizer::callback(const sensor_msgs::PointCloud2::ConstPtr& ori_pointcloud, const sensor_msgs::Imu::ConstPtr& ori_imu){
    // cout << "*******************" << endl;
    sensor_msgs::PointCloud2 syn_pointcloud = *ori_pointcloud;
    sensor_msgs::Imu syn_imu = *ori_imu;

    ROS_INFO("In sync, pointcloud stamp value is: %f", syn_pointcloud.header.stamp.toSec());
    // ROS_INFO("imu stamp value is: %f", syn_imu.header.stamp.toSec());

    pubVelodyne.publish(syn_pointcloud);
    pubImu.publish(syn_imu);
}

mySynchronizer::mySynchronizer()
{
    ros::NodeHandle nh;
    pubVelodyne = nh.advertise<sensor_msgs::PointCloud2>("/sync/velodyne_points", 1);
    pubImu = nh.advertise<sensor_msgs::Imu>("/sync/imu/data", 1);

    message_filters::Subscriber<sensor_msgs::Imu> imu_sub(nh, "/imu/data", 1);
    message_filters::Subscriber<sensor_msgs::PointCloud2> velodyne_sub(nh, "/velodyne_points", 1);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::Imu> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), velodyne_sub, imu_sub);
    sync.registerCallback(boost::bind(&mySynchronizer::callback, this, _1, _2));

    ros::spin();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "msg_synchronizer");
    ROS_INFO("\033[1;32m---->\033[0m Sync msgs node Started.");

    mySynchronizer wode;

    // ros::spin();

    return 0;
}