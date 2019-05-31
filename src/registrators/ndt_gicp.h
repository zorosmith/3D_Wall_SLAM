#ifndef REGISTRATORS_NDT_GICP_H_
#define REGISTRATORS_NDT_GICP_H_

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>
#include <pcl/registration/gicp.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "registrator_interface.h"

#define NDT_USE_GPU

#ifdef NDT_USE_GPU
#include "ndt_gpu/NormalDistributionsTransform.h"
#else
#include <pcl/registration/ndt.h>
#endif

//#include "libicp/icpPointToPlane.h"
//#include "libicp/icpPointToPoint.h"

#include <ctime>
#include <iostream>
#include <vector>
#include <cmath>
#include "common/macro_defines.h"

namespace l2l_calib{
namespace registrator{
  
template <typename PointType>
class RegistrationWithNDTandGICP
 : public RegistratorInterface<PointType>
{
  using typename 
    RegistratorInterface<PointType>::PointCloudSource;
  using typename 
    RegistratorInterface<PointType>::PointCloudTarget;
  using typename 
    RegistratorInterface<PointType>::PointCloudSourcePtr;
  using typename 
    RegistratorInterface<PointType>::PointCloudTargetPtr;
  
public:
  
  RegistrationWithNDTandGICP( bool using_voxel_filter = true, 
                              double target_resolution = 0.000001,
                              double source_resolution = 0.05)
    : input_cloud_( new PointCloudSource )
    , target_cloud_( new PointCloudTarget )
    , down_sampled_input_cloud_( new PointCloudSource )
    , down_sampled_target_cloud_( new PointCloudTarget )
    , target_resolution_( target_resolution )
    , source_resolution_( source_resolution )
    , using_voxel_filter_( using_voxel_filter )
  {
    RegistratorInterface<PointType>::type_ = kNdtWithGicp;
    //设置依赖尺度NDT参数
    //为终止条件设置最小转换差异
    ndt_.setTransformationEpsilon(0.01);
    //为More-Thuente线搜索设置最大步长
    ndt_.setStepSize( 0.1 );
    //设置NDT网格结构的分辨率（VoxelGridCovariance）
    ndt_.setResolution(0.7);
    //设置匹配迭代的最大次数
    ndt_.setMaximumIterations(120);
    
    gicp_.setRotationEpsilon(1e-3);
    gicp_.setMaximumIterations(2);
    
    approximate_target_filter_.setLeafSize(target_resolution_, 
      target_resolution_, target_resolution_);
    approximate_source_filter_.setLeafSize(source_resolution_, 
    source_resolution_, source_resolution_);
  }
  
  ~RegistrationWithNDTandGICP() = default;
  
  typedef boost::shared_ptr< RegistrationWithNDTandGICP<PointType> > 
    Ptr;
  typedef boost::shared_ptr< const RegistrationWithNDTandGICP<PointType> > 
    ConstPtr;
  
  inline void
  setInputSource (const PointCloudSourcePtr &cloud) override
  {
    if (cloud->points.empty ()) {
      PCL_ERROR ("Invalid or empty point cloud dataset given!\n");
      return;
    }
    *input_cloud_ = *cloud;
  }
  
  inline void
  setInputTarget (const PointCloudTargetPtr &cloud) override
  {
    if (cloud->points.empty ()) {
      PCL_ERROR ("Invalid or empty point cloud dataset given!\n");
      return;
    }
    *target_cloud_ = *cloud;
  }
  
  bool align (const Eigen::Matrix4f& guess, Eigen::Matrix4f& result)
  {
    if( using_voxel_filter_ )
    {
      approximate_source_filter_.setInputCloud( input_cloud_ );
      approximate_source_filter_.filter(*down_sampled_input_cloud_);
      
      approximate_target_filter_.setInputCloud( target_cloud_ );
      approximate_target_filter_.filter(*down_sampled_target_cloud_);
    }
    else
    {
      *down_sampled_input_cloud_ = *input_cloud_;
      *down_sampled_target_cloud_ = *target_cloud_;
    }
    
    //*******gordon code********
    // pcl::io::savePCDFileASCII ("/home/gordon/fase_ws/src/ddd_wall_mapping/target.pcd",
    //                            *down_sampled_target_cloud_);
    // pcl::io::savePCDFileASCII ("/home/gordon/fase_ws/src/ddd_wall_mapping/source.pcd",
    //                            *down_sampled_input_cloud_);
    // std::cout << "In align function in ndt_gicp.h , DS target size: " 
    //         << down_sampled_target_cloud_->points.size() << std::endl;
    // std::cout << "In align function in ndt_gicp.h , DS source size: " 
    //         << down_sampled_input_cloud_->points.size() << std::endl;
    //*******gordon code********

    PointCloudSourcePtr output_cloud(new PointCloudSource);
    
    Eigen::Matrix4f ndt_guess = guess;
    double ndt_score = 0.9;

    clock_t start_ndt, end_ndt;

    if( use_ndt_ )
    {
      ndt_.setInputSource(down_sampled_input_cloud_);
      ndt_.setInputTarget(down_sampled_target_cloud_);
      
      start_ndt = clock();

      #ifdef NDT_USE_GPU
      ndt_.align( guess );
      #else
      ndt_.align(*output_cloud, guess);
      #endif

      end_ndt = clock();
      std::cout << "ndt align cost time : " << (end_ndt - start_ndt)/1000 << std::endl;
      
      ndt_score = ndt_.getFitnessScore();
      
      #ifdef NDT_USE_GPU
      ndt_score /= 20;
      #endif

      ndt_guess = ndt_.getFinalTransformation();
    }

    double icp_score = 10.;
    Eigen::Matrix4f final_guess;
    clock_t start_Gicp, end_Gicp;
    if( ndt_score <= 1. )
    {  
      icp_score = ndt_score;
      
      gicp_.setInputSource( down_sampled_input_cloud_ );
      gicp_.setInputTarget( down_sampled_target_cloud_ );
      //*******gordon code********
      output_cloud->clear();
      //*******gordon code********

      start_Gicp = clock();
      gicp_.align(*output_cloud, ndt_guess);
      end_Gicp = clock();
      std::cout << "Gicp cost time : " << (end_Gicp-start_Gicp)/1000 << std::endl;
      
      icp_score = gicp_.getFitnessScore();
      final_guess = gicp_.getFinalTransformation();
      
      // final_guess(2,3) = 0.;     

      RegistratorInterface<PointType>::final_score_ = std::exp( -icp_score );
      result = final_guess;

      return true;
    }
    else 
    {
      result = guess;
      final_score_ = std::exp( -icp_score );
      return false;
    }
    
    //*******gordon code********
    std::cout << "In ndt_gicp.h, ndt_score : " << ndt_score << std::endl;
    std::cout << "In ndt_gicp.h, icp_score : " << icp_score << std::endl;
    //*******gordon code********

    return true;
  }
  
  inline 
  double getFitnessScore() { return final_score_; }
  
  inline
  void enableNdt( bool use_ndt ){ use_ndt_ = use_ndt; }
  
private:
#ifdef NDT_USE_GPU
  gpu::GNormalDistributionsTransform ndt_;
#else 
  pcl::NormalDistributionsTransform<PointType, PointType> ndt_;
#endif
  pcl::GeneralizedIterativeClosestPoint<PointType, PointType> gicp_;
  
  PointCloudSourcePtr input_cloud_;
  PointCloudTargetPtr target_cloud_;
  
  PointCloudSourcePtr down_sampled_input_cloud_;
  PointCloudTargetPtr down_sampled_target_cloud_;
  
  // pcl::ApproximateVoxelGrid<PointType> approximate_voxel_filter_;
  //*******gordon code*******
  pcl::ApproximateVoxelGrid<PointType> approximate_target_filter_;
  pcl::ApproximateVoxelGrid<PointType> approximate_source_filter_;
  //*******gordon code*******

  //double voxel_resolution_;
  //*******gordon code*******
  double target_resolution_;
  double source_resolution_;
  //*******gordon code*******

  bool using_voxel_filter_;
  bool use_ndt_ = true;
  
  double final_score_ = 0.;
};
}  // namespace registrator
}  // namespace l2l_calib

#endif // REGISTRATORS_NDT_GICP_H_