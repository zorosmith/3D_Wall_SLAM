#include <string>
#include "lidar.h"
#include "ros/ros.h"
#include "registrators/ndt_gicp.h"

#include <pcl_ros/point_cloud.h>
//#include <pcl_conversions/pcl_conversions.h>

#include <tf_conversions/tf_eigen.h>
#include <tf/transform_broadcaster.h>

using namespace l2l_calib;
using namespace l2l_calib::common;

int ndtCount = 0;
Eigen::Matrix4f ndtTransform = Eigen::Matrix4f::Identity();

void publishTF(tf::StampedTransform& laserOdom ){
    Eigen::Quaterniond eigen_q(ndtTransform.block<3,3>(0,0).cast<double>());
    Eigen::Vector3d eigen_T(ndtTransform.block<3,1>(0,3).cast<double>());
    laserOdom.setRotation(tf::Quaternion(eigen_q.x(), eigen_q.y(), eigen_q.z(), eigen_q.w()));
    laserOdom.setOrigin(tf::Vector3(eigen_T(0),eigen_T(1),eigen_T(2)));
    laserOdom.stamp_ = ros::Time::now();
    
}


int main(int argc, char **argv){
    ros::init(argc, argv, "ndt_map");
    ros::NodeHandle n;

    ros::Publisher pubMapCloud;
    pubMapCloud = n.advertise<sensor_msgs::PointCloud2>("/mapCloud", 1);

    std::vector<Lidar::PointCloudPtr> vec_PointCloudPtr;
    Lidar::PointCloudPtr mapPointCloud;

    pcl::ApproximateVoxelGrid<Lidar::PointType> map_voxel_filter;
    map_voxel_filter.setLeafSize(0.05, 0.05, 0.05);

    tf::TransformBroadcaster tfBroadcaster;
    tf::StampedTransform laserOdometryTrans;
    laserOdometryTrans.frame_id_ = "laser_init";
    laserOdometryTrans.child_frame_id_ = "laser";

    // construct a Lidar instance
    std::shared_ptr<registrator::RegistratorInterface<Lidar::PointType>>
        ndt_mapping_;
    std::shared_ptr<Lidar> lidar_;
    Eigen::Matrix4f tf_pose = Eigen::Matrix4f::Identity();
    std::string topic_name = "/velodyne_points";
    lidar_ = std::make_shared<Lidar>( tf_pose );
    lidar_->Initialise( n, topic_name );  //初始化之后便开始订阅imu数据

    // construct a RegistrationWithNDTandGICP instance
    ndt_mapping_ = std::make_shared<
    registrator::RegistrationWithNDTandGICP<Lidar::PointType>>();

    ros::Rate loop_rate(10);
    while( ros::ok() ){
        ros::spinOnce();
        if( !lidar_->GotFirstData() )
            continue;

        Lidar::PointCloudPtr targetCloudPtr, sourceCloudPtr;
        targetCloudPtr = lidar_->GetNewCloud();
        sourceCloudPtr = lidar_->GetSourceCloud();
        std::cout << "targetCloud size" << targetCloudPtr->points.size() << std::endl;
        std::cout << "sourceCloud size" << sourceCloudPtr->points.size() << std::endl;
        ndt_mapping_->setInputTarget(targetCloudPtr);
        ndt_mapping_->setInputSource(sourceCloudPtr);
        Eigen::Matrix4f guess = Eigen::Matrix4f::Identity();
        Eigen::Matrix4f result = Eigen::Matrix4f::Identity();
        if (ndt_mapping_->align( guess, result )){
            // calculate odometry
            if (ndtCount == 0)
                ndtTransform = result;
            else
                ndtTransform = ndtTransform * result;

            // publish TF
            publishTF(laserOdometryTrans);
            tfBroadcaster.sendTransform(laserOdometryTrans);

            // transform pointcloud for mapping
            pcl::transformPointCloud (*sourceCloudPtr, *sourceCloudPtr, ndtTransform);
            if (ndtCount == 0)
                vec_PointCloudPtr.push_back(targetCloudPtr);
                *mapPointCloud += *targetCloudPtr;

            vec_PointCloudPtr.push_back(sourceCloudPtr);
            *mapPointCloud += *sourceCloudPtr;
            std::cout << "vec_PointCloudPtr size : " << vec_PointCloudPtr.size() << std::endl;

            // publishGlobalMap();
            map_voxel_filter.setInputCloud(mapPointCloud);
            map_voxel_filter.filter(*mapPointCloud);

            sensor_msgs::PointCloud2 cloudMsgTemp;
            pcl::toROSMsg(*mapPointCloud,cloudMsgTemp);
            cloudMsgTemp.header.stamp = ros::Time().now();
            cloudMsgTemp.header.frame_id = "/laser_init";
            pubMapCloud.publish(cloudMsgTemp);
        }

        ndtCount++;

        loop_rate.sleep();
    }

    return 0;
}
