#include "lidar.h"
#include "my_imu.h"
#include "ros/ros.h"
#include "registrators/ndt_gicp.h"
#include "extr_PlaneFeature.h"


// #include <pcl/conversions.h>
// #include <pcl_conversions/pcl_conversions.h>

#include <tf/transform_broadcaster.h>

#include <string>

using namespace l2l_calib;

class NdtMapping{
    public:
        tf::TransformBroadcaster tfBroadcaster;
        tf::StampedTransform laserOdometryTrans;

        ros::Publisher pubGlobalMap;

        pcl::VoxelGrid<Lidar::PointType> voxel_filter;

        NdtMapping();
        ~NdtMapping(){};

        void initializationValue();
        void publishTF();
        void calculate_odom(Lidar::PointCloudPtr targetPtr, Lidar::PointCloudPtr sourcePtr);
        void publishMapCloud();
        void mapping(Lidar::PointCloudPtr targetCloud, Lidar::PointCloudPtr sourceCloud);
        Eigen::Matrix4f imuPreintegration(double lidar_stamp);
        void run();

    private:
        std::shared_ptr<Lidar> lidar_;
        std::shared_ptr<My_Imu> imu_;
        std::shared_ptr<Extr_PLANEFEATURE> ExtrPlanes_;
        std::shared_ptr<registrator::RegistrationWithNDTandGICP<Lidar::PointType>>
            scan_matcher_;
        // std::shared_ptr<Imu> imu_;

        int ndtCount;
        bool match_success;

        std::string lidar_topic_name;
        std::string imu_topic_name;
        Eigen::Matrix4f tf_pose;

        Eigen::Matrix4f guess_mat4f;
        Eigen::Matrix4f result;
        std::vector<Eigen::Matrix4f> vec_pairwiseResult;
        Eigen::Matrix4f ndtTransform;

        bool GUESS;
        
        // std::vector<Lidar::PointCloudPtr> vec_PointCloudPtr;
        Lidar::PointCloudPtr mapPointCloud;

};

NdtMapping::NdtMapping(){
    ros::NodeHandle nh;
    pubGlobalMap = nh.advertise<sensor_msgs::PointCloud2>("/globalMap", 2);

    initializationValue();

    lidar_ = std::make_shared<Lidar>( tf_pose );
    lidar_->Initialise( nh, lidar_topic_name );  //初始化之后便开始订阅lidar数据

    imu_ = std::make_shared<My_Imu>( tf_pose );
    imu_->Initialise( nh, imu_topic_name );  //初始化之后便开始订阅imu数据

    ExtrPlanes_ = std::make_shared<Extr_PLANEFEATURE>();
    ExtrPlanes_->Initialise( nh, lidar_topic_name );  // 订阅同步后的lidar数据

    scan_matcher_ = std::make_shared<
    registrator::RegistrationWithNDTandGICP<Lidar::PointType>>();
}

void NdtMapping::initializationValue(){
    GUESS = false;

    ndtCount = 0;
    match_success = false;
    mapPointCloud.reset(new Lidar::PointCloudType);
    mapPointCloud->clear();

    ndtTransform = Eigen::Matrix4f::Identity();
    tf_pose = Eigen::Matrix4f::Identity();
    lidar_topic_name = "/sync/velodyne_points";
    imu_topic_name = "/sync/imu/data";

    laserOdometryTrans.frame_id_ = "/laser_init";
    laserOdometryTrans.child_frame_id_ = "/laser";

    voxel_filter.setLeafSize(0.03, 0.03, 0.03);
}

void NdtMapping::publishTF(){
    Eigen::Quaterniond eigen_q(ndtTransform.block<3,3>(0,0).cast<double>());
    Eigen::Vector3d eigen_T(ndtTransform.block<3,1>(0,3).cast<double>());
    laserOdometryTrans.setRotation(tf::Quaternion(eigen_q.x(), eigen_q.y(), eigen_q.z(), eigen_q.w()));
    laserOdometryTrans.setOrigin(tf::Vector3(eigen_T(0),eigen_T(1),eigen_T(2)));
    laserOdometryTrans.stamp_ = ros::Time::now();
    tfBroadcaster.sendTransform(laserOdometryTrans);    
}

void NdtMapping::calculate_odom(Lidar::PointCloudPtr targetPtr, Lidar::PointCloudPtr sourcePtr){
    scan_matcher_->setInputTarget(targetPtr);
    scan_matcher_->setInputSource(sourcePtr);

    // std::cout << "In calculate_odom function !" << std::endl;
    // std::cout << "targetCloud size : " << targetPtr->points.size() << std::endl;
    // std::cout << "sourceCloud size : " << sourcePtr->points.size() << std::endl;

    if (scan_matcher_->align( guess_mat4f, result )){
        vec_pairwiseResult.push_back(result);
        std::cout << "NDT T matrix : " << std::endl << result << std::endl;
        if (ndtCount == 0)
            ndtTransform = result;
        else
            ndtTransform = ndtTransform * result;

        // publish TF
        publishTF();

        match_success = true;
    }
    else{
        std::cout << "Fail to match pointclouds ! " << std::endl;
        sourcePtr = lidar_->GetNewCloud(); // 将无法正确匹配的sourceCloud去掉
        sourcePtr = lidar_->GetSourceCloud();
        if (sourcePtr == 0)
            return;

        calculate_odom(targetPtr, sourcePtr);
    }
}

void NdtMapping::publishMapCloud(){
    sensor_msgs::PointCloud2 cloudMsgTemp;
    pcl::toROSMsg(*mapPointCloud,cloudMsgTemp);
    cloudMsgTemp.header.stamp = ros::Time().now();
    cloudMsgTemp.header.frame_id = "/laser_init";
    pubGlobalMap.publish(cloudMsgTemp);
}

void NdtMapping::mapping(Lidar::PointCloudPtr targetCloud, Lidar::PointCloudPtr sourceCloud){
    // std::cout << "In mapping function !" << std::endl;
    // std::cout << "ndtTransform: " << std::endl;
    // std::cout << ndtTransform << std::endl;

    pcl::transformPointCloud (*sourceCloud, *sourceCloud, ndtTransform);

    if (ndtCount == 0){
        // vec_PointCloudPtr.push_back(targetCloud);
        *mapPointCloud += *targetCloud;
    }
    // std::cout << "push target, mapPointCloud size : " << mapPointCloud->points.size() << std::endl;

    // vec_PointCloudPtr.push_back(sourceCloud);
    *mapPointCloud += *sourceCloud;
    // std::cout << "push source, mapPointCloud size : " << mapPointCloud->points.size() << std::endl;


    // std::cout << "vec_PointCloudPtr size : " << vec_PointCloudPtr.size() << std::endl;
    // std::cout << "before voxel_filter, mapPointCloud size : " << mapPointCloud->points.size() << std::endl;
    Lidar::PointCloudPtr filter_mapPointCloud(new Lidar::PointCloudType);
    voxel_filter.setInputCloud(mapPointCloud);
    voxel_filter.filter(*filter_mapPointCloud);
    mapPointCloud->clear();
    *mapPointCloud += *filter_mapPointCloud;
    // std::cout << "after voxel_filter, mapPointCloud size : " << mapPointCloud->points.size() << std::endl;

    publishMapCloud();
}

Eigen::Matrix4f NdtMapping::imuPreintegration(double lidar_stamp){
    Eigen::Matrix3f imu_rotMat = imu_->GETimu_rotMat(lidar_stamp);
    // This function is uncompleted
    return Eigen::Matrix4f::Identity();
}

void NdtMapping::run(){
    if (lidar_->Get_Clouds_size() < 3)
        return;

    Lidar::PointCloudPtr targetCloudPtr, sourceCloudPtr;
    targetCloudPtr = lidar_->GetNewCloud();
    sourceCloudPtr = lidar_->GetSourceCloud();
    if (targetCloudPtr == 0 || sourceCloudPtr == 0)
        return;

    // std::cout << "**************************" << std::endl;
    // std::cout << "targetCloud size : " << targetCloudPtr->points.size() << std::endl;
    // std::cout << "sourceCloud size : " << sourceCloudPtr->points.size() << std::endl;
    double lidar_stamp = lidar_ -> Get_SourcePoints_Stamp();
    if (lidar_stamp < 0)
        GUESS = false;

    if( GUESS ){
        guess_mat4f = imuPreintegration(lidar_stamp);
    }
    else{
        guess_mat4f = Eigen::Matrix4f::Identity();
    }
    result = Eigen::Matrix4f::Identity();

    // calculate odometry
    calculate_odom(targetCloudPtr, sourceCloudPtr);
    // std::cout << "Get out of calculate_odom function !" << std::endl;
    // std::cout << "ndtTransform: " << std::endl;
    // std::cout << ndtTransform << std::endl;

    ExtrPlanes_->run_extraction();
    ExtrPlanes_->planeCorrespondence(vec_pairwiseResult[0]);

    if (match_success){
        mapping(targetCloudPtr, sourceCloudPtr);
        match_success = false;
        ndtCount++;
    }
        
}

int main(int argc, char **argv){
    ros::init(argc, argv, "ndt_map");
    ROS_INFO("\033[1;32m---->\033[0m ndt map node Started.");

    NdtMapping ndt;

    ros::Rate rate(10);
    while (ros::ok())
    {
        ros::spinOnce();

        ndt.run();

        rate.sleep();
    }

    return 0;
}