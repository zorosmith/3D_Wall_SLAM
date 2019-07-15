#ifndef MY_POINT_CLOUD_H
#define MY_POINT_CLOUD_H

#include <vector>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/common/centroid.h>

/*
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef typename PointCloud::Ptr PointCloudPtr;
*/

using PointType = pcl::PointXYZI;
using PointCloudType = pcl::PointCloud<pcl::PointXYZI>;
using PointCloudPtr = PointCloudType::Ptr;

using PointIndices = pcl::PointIndices;

#endif /*MY_POINT_CLOUD_H*/
