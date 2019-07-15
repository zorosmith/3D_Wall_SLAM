#include "extr_PlaneFeature.h"


// bool Extr_PLANEFEATURE::sortIndices(const PointIndices& indice1, const PointIndices& indice2)
bool sortIndices(const PointIndices& indice1, const PointIndices& indice2)
{
    return (indice1.indices.size() > indice2.indices.size());
}

void Extr_PLANEFEATURE::Initialise( 
    ros::NodeHandle& nh, std::string topic_name )
{
    subscriber_ = nh.subscribe(topic_name, 100, &Extr_PLANEFEATURE::Callback, this );
    pubProcessCloud_A = nh.advertise<sensor_msgs::PointCloud2>("/EuclideanCloud", 10);
    pubProcessCloud_B = nh.advertise<sensor_msgs::PointCloud2>("/SegCloud", 10);
    pubProcessCloud_C = nh.advertise<sensor_msgs::PointCloud2>("/PlanesCloud", 10);
    pubCentroidMarker = nh.advertise<visualization_msgs::MarkerArray>("/CentroidMarker", 10);

    laserCount = -1;
    sampleNum = 5;
    got_first_data_ = false;

    inPointCloud_ptr = boost::make_shared<PointCloudType>();
    outPointCloud_ptr = boost::make_shared<PointCloudType>();
}

void Extr_PLANEFEATURE::Callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    laserCount++;
    // std::cout << "in lidar.cc, laserCount : " << laserCount << std::endl;
    if( laserCount % sampleNum > 0)
        return;

    if( !got_first_data_ ) {
        got_first_data_ = true;
    }

    double timestamp = msg->header.stamp.toSec() - 1560860000.0;
    timestamp  = int(timestamp * 1000) / 1000.0;
    ROS_INFO("In planeFeature, pointcloud stamp value is: %f", timestamp);

    PointCloudPtr incoming_cloud( new PointCloudType );
    pcl::fromROSMsg( *msg, *incoming_cloud );

    AddCloud( incoming_cloud );
}

void Extr_PLANEFEATURE::publishEuclideanCloud(PointCloudPtr inPtr){
    sensor_msgs::PointCloud2 cloudMsgTemp;
    pcl::toROSMsg(*inPtr,cloudMsgTemp);
    cloudMsgTemp.header.stamp = ros::Time().now();
    cloudMsgTemp.header.frame_id = "/EuclideanLink";
    pubProcessCloud_A.publish(cloudMsgTemp);
}

void Extr_PLANEFEATURE::publishSegCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inPtr)
{
    sensor_msgs::PointCloud2 cloudMsgTemp;
    pcl::toROSMsg(*inPtr,cloudMsgTemp);
    cloudMsgTemp.header.stamp = ros::Time().now();
    cloudMsgTemp.header.frame_id = "/SegLink";
    pubProcessCloud_B.publish(cloudMsgTemp);
}
   
void Extr_PLANEFEATURE::publishPlanesCloud(PointCloudPtr inPtr)
{
    sensor_msgs::PointCloud2 cloudMsgTemp;
    pcl::toROSMsg(*inPtr,cloudMsgTemp);
    cloudMsgTemp.header.stamp = ros::Time().now();
    cloudMsgTemp.header.frame_id = "/PlanesLink";
    pubProcessCloud_C.publish(cloudMsgTemp);
}

void Extr_PLANEFEATURE::publishCentroidMarker(std::vector<Eigen::Vector4f> inCentroidVec)
{
    if (inCentroidVec.empty())
    {
        std::cout << "publishCentroidMarker: input vector is empty!" << std::endl;
        return;
    }

    visualization_msgs::MarkerArray markerArray;
    for(int i = 0; i<inCentroidVec.size(); i++)
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "SegLink";
        marker.header.stamp = ros::Time().now();
        marker.ns = "Plane_Centroid";
        marker.id = i;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = inCentroidVec[i](0);
        marker.pose.position.y = inCentroidVec[i](1);
        marker.pose.position.z = inCentroidVec[i](2);
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.05;
        marker.scale.y = 0.05;
        marker.scale.z = 0.05;
        marker.color.a = 1.0; // Don't forget to set the alpha!
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.lifetime = ros::Duration(1.0);
        markerArray.markers.push_back(marker);
    }
    
    pubCentroidMarker.publish(markerArray);
}

void Extr_PLANEFEATURE::Voxel_Grid()
{
    float leafsize = atof( pr.getData( "leaf_size" ).c_str());

    pcl::VoxelGrid<PointType> vg;
    // PointCloudPtr outPointCloud_ptr (new PointCloudType);
    vg.setInputCloud (inPointCloud_ptr);
    vg.setLeafSize (leafsize, leafsize, leafsize);
    vg.filter (*outPointCloud_ptr);
    std::cout << "After VoxelGrid filtering has: " << outPointCloud_ptr->points.size ()  << " points." << std::endl;
}

void Extr_PLANEFEATURE::Extract_Indices (PointCloudPtr input_pc, PointCloudPtr output_pc,
                                        PointIndices::Ptr input_indices, bool Neg_flag, bool Keep_flag)
{
    pcl::ExtractIndices<PointType> extract;
    extract.setInputCloud (input_pc);
    extract.setIndices (input_indices);
    extract.setNegative (Neg_flag); // if false, get the plane points; if true, get points except for plane points
    extract.setKeepOrganized (Keep_flag); // keep the order of indices; the order would be changed once the poincloud being trimmed 
    extract.filter (*output_pc);   
}


void Extr_PLANEFEATURE::Euclidean_Cluster_Extraction()
{
    float euler_tolerance = atof(pr.getData( "cluster_tolerance" ).c_str());
    float euclidean_ratio = atof(pr.getData("EuclideanRatio").c_str());
    int min_cluster = atoi(pr.getData("MinCluster").c_str());

    std::vector<PointIndices> euclideanIndices;
    pcl::search::KdTree<PointType>::Ptr tree (new pcl::search::KdTree<PointType>);
    tree->setInputCloud (inPointCloud_ptr);

    pcl::EuclideanClusterExtraction<PointType> ec;
    ec.setClusterTolerance (euler_tolerance);
    ec.setMinClusterSize (min_cluster);
    ec.setMaxClusterSize (25000000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (inPointCloud_ptr);
    ec.extract (euclideanIndices);

    std::sort( euclideanIndices.begin(), euclideanIndices.end(), sortIndices );

    int size_euclidean = euclideanIndices.size();
    std::cout << "Euclidean Cluster size : " << size_euclidean << std::endl;
    for (int i = 0; i < std::ceil(size_euclidean*euclidean_ratio); i++)
    {
        PointCloudPtr euclidean_cloudPtr(new PointCloudType);
        PointIndices::Ptr euclidean_indicesPtr = boost::make_shared< PointIndices >(euclideanIndices[i]);
        std::cout << "indices size : " << euclidean_indicesPtr->indices.size() << std::endl;
        Extract_Indices (inPointCloud_ptr, euclidean_cloudPtr, euclidean_indicesPtr, false);
        *outPointCloud_ptr += *euclidean_cloudPtr; 
    }
}

void Extr_PLANEFEATURE::Region_Growing(std::vector<PointCloudPtr>& vec_segCloud,
                                        std::vector<PointIndices>& vec_segIndices)
{
    int MinClusterSize = atoi(pr.getData("MinClusterSize").c_str());
    float seg_ratio = atof(pr.getData("SegRatio").c_str());
    float smoothness = atof(pr.getData("Smoothness").c_str());
    float curvature = atof(pr.getData("Curvature").c_str());

    pcl::search::Search<PointType>::Ptr tree = boost::shared_ptr<pcl::search::Search<PointType> > 
        (new pcl::search::KdTree<PointType>);
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
    pcl::NormalEstimation<PointType, pcl::Normal> normal_estimator;
    normal_estimator.setSearchMethod (tree);
    // normals are consistently oriented towards the viewpoint
    normal_estimator.setViewPoint (std::numeric_limits<float>::max (), std::numeric_limits<float>::max (), std::numeric_limits<float>::max ());
    normal_estimator.setInputCloud (inPointCloud_ptr);
    normal_estimator.setKSearch (50);
    normal_estimator.compute (*normals);

    pcl::RegionGrowing<PointType, pcl::Normal> reg;
    reg.setMinClusterSize (MinClusterSize);
    reg.setMaxClusterSize (1000000);
    reg.setSearchMethod (tree);
    reg.setNumberOfNeighbours (50);
    reg.setInputCloud (inPointCloud_ptr);
    //reg.setIndices (indices);   对经过直通滤波的部分点云进行处理，不需要对全局点云进行处理
    reg.setInputNormals (normals);
    reg.setSmoothnessThreshold (smoothness / 180.0 * M_PI);
    reg.setCurvatureThreshold (curvature);
    reg.extract (vec_segIndices);
    
    // visualize
    pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();
    publishSegCloud(colored_cloud);
    // pcl::visualization::CloudViewer viewer_RG ("RegionGrowing");
    // viewer_RG.showCloud(colored_cloud);
    // while (!viewer_RG.wasStopped ())
    // {
    // }

    // Extract point clouds from PointIndices
    std::sort( vec_segIndices.begin(), vec_segIndices.end(), sortIndices );

    int size_vec_segIndices = vec_segIndices.size();
    std::cout << "region growing size : " << size_vec_segIndices << std::endl;
    for (int i = 0; i < std::ceil(size_vec_segIndices*seg_ratio); i++)
    {
        PointCloudPtr seg_cloudPtr(new PointCloudType);
        PointIndices::Ptr seg_indicesPtr = boost::make_shared< PointIndices >(vec_segIndices[i]);
        std::cout << "seg_indicesPtr size : " << seg_indicesPtr->indices.size() << std::endl;
        Extract_Indices (inPointCloud_ptr, seg_cloudPtr, seg_indicesPtr, false);
        vec_segCloud.push_back(seg_cloudPtr);
        // std::cout << "region grow, point cloud adress : " << seg_cloudPtr << std::endl;
    }
}

void Extr_PLANEFEATURE::Extract_from_indices(PointIndices &output, PointIndices inputIndices, PointIndices input)
{
    for (std::vector<int>::const_iterator pit = inputIndices.indices.begin (); pit != inputIndices.indices.end (); ++pit)
        output.indices.push_back(input.indices[*pit]);
}

void Extr_PLANEFEATURE::Ransac_Plane(std::vector<PointCloudPtr>& vec_clouds,
                                    std::vector<pcl::ModelCoefficients>& vec_coefficients,
                                    PointIndices& segIndices,
                                    PointCloudType& EuclideanCloud)
{
    float distThres = atof( pr.getData( "DistanceThreshold" ).c_str() );
    float remain_scale = atof( pr.getData("RemainScale").c_str() );

    std::vector<PointIndices> vec_planeIndices;

    pcl::SACSegmentation<PointType> SACSeg;
    SACSeg.setOptimizeCoefficients (true);
    SACSeg.setModelType (pcl::SACMODEL_PLANE);
    SACSeg.setMethodType (pcl::SAC_RANSAC);
    SACSeg.setMaxIterations (100);
    SACSeg.setDistanceThreshold (distThres);

    int remainPoints = remain_scale * inPointCloud_ptr->points.size();
    // std::cout << "remainPoints : " << remainPoints << std::endl;
    while(inPointCloud_ptr->points.size() > remainPoints)
    {
        PointIndices::Ptr segIndicesPtr (new PointIndices);
        PointIndices ori_segIndices;
        pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
        SACSeg.setInputCloud (inPointCloud_ptr);
        SACSeg.segment (*segIndicesPtr, *coefficients);
        if(segIndicesPtr->indices.size() == 0)
        {
            std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
        }
        std::cout << "plane size : " << segIndicesPtr->indices.size() << std::endl;

        //store point indices , cofficient
        Extract_from_indices(ori_segIndices, *segIndicesPtr, segIndices);
        vec_planeIndices.push_back(ori_segIndices);
        vec_coefficients.push_back(*coefficients);

        // Remove plane point cloud indices, extract the rest
        // std::cout << "remain point cloud size : " << inPointCloud_ptr->points.size() << std::endl;
        pcl::PointCloud< PointType >::Ptr remainPtr (new PointCloudType);
        Extract_Indices(inPointCloud_ptr, remainPtr, segIndicesPtr, true); 
        inPointCloud_ptr->clear();
        *inPointCloud_ptr = *remainPtr;
    }

    int size_vec_planeIndices = vec_planeIndices.size();
    for(int i = 0; i < size_vec_planeIndices; i++)
    {
        PointCloudPtr planePtr(new PointCloudType);
        PointIndices::Ptr planeIndicesPtr = boost::make_shared<PointIndices>(vec_planeIndices[i]);
        PointCloudPtr tempPtr = boost::make_shared<PointCloudType>(EuclideanCloud);
        Extract_Indices(tempPtr, planePtr, planeIndicesPtr, false);
        vec_clouds.push_back(planePtr);
    }
}

void Extr_PLANEFEATURE::run_extraction()
{
    if(!got_first_data_)
        return;

    // inPointCloud_ptr->clear();
    // outPointCloud_ptr->clear();
    inPointCloud_ptr = boost::make_shared< PointCloudType >( clouds_[0] );
    std::cout << "Before voxelGrid, inpointCloud_ptr size: "<< inPointCloud_ptr->points.size() << std::endl;
    Voxel_Grid();
    inPointCloud_ptr->swap(*outPointCloud_ptr);
    std::cout << "inPtr : " << inPointCloud_ptr->points.size() << "; outPtr : " << outPointCloud_ptr->points.size() << std::endl;
    outPointCloud_ptr->clear();

    Euclidean_Cluster_Extraction();
    publishEuclideanCloud(outPointCloud_ptr);
    std::cout << "After Euclidean cluster, point cloud size : " << outPointCloud_ptr->points.size() << std::endl;
    PointCloudType euclideanCloud = *outPointCloud_ptr;  // 留到提取平面时使用
    inPointCloud_ptr->swap(*outPointCloud_ptr);
    outPointCloud_ptr->clear();
    std::cout << "Euclidean cluster ok " << std::endl;

    std::vector<PointCloudPtr> seg_cloudVec;
    std::vector<PointIndices> seg_indicesVec;   // use in Ransac
    Region_Growing(seg_cloudVec, seg_indicesVec);
    // for(int i = 0; i < seg_cloudVec.size(); i++)
    // {
    //     std::cout << "In run, point cloud cluster adress : " << seg_cloudVec[i] << std::endl;
    // }
    std::cout << "region growing ok " << std::endl;

    std::vector<PointCloudPtr> ransac_planeClouds;
    std::vector<pcl::ModelCoefficients> ransac_planeCoefficients;
    int size_seg_cloudVec = seg_cloudVec.size();
    for (int i = 0; i < size_seg_cloudVec; i++)
    {
        inPointCloud_ptr->clear();
        inPointCloud_ptr = seg_cloudVec[i];
        Ransac_Plane(ransac_planeClouds, ransac_planeCoefficients, seg_indicesVec[i], euclideanCloud); // 需要提取壁面点云，计算壁面质心
    }
    // std::cout << "euclideanCloud size : " << euclideanCloud.points.size() << std::endl;
    std::cout << "ransac ok " << std::endl;

    int size_plane_clouds = ransac_planeClouds.size();
    std::vector<Eigen::Vector4f> vec_centerG;
    for (int j = 0; j < size_plane_clouds; j++)
    {
        Eigen::Vector4f centerG;
        pcl::compute3DCentroid(*(ransac_planeClouds[j]), centerG);
        vec_centerG.push_back(centerG);
    }
    std::cout << "compute 3D centroid ok " << std::endl;

    int size_vec_centerG = vec_centerG.size();
    if (size_plane_clouds != size_vec_centerG)
    {
        std::cout << "Error!!! size_plane_clouds != size_vec_centerG !!!" << std::endl;
        return;
    }

    std::cout << "How many planes : " << size_vec_centerG << std::endl;
    for (int k = 0; k < size_vec_centerG; k++)
    {
        Plane_Feature pF;

        pF.size = ransac_planeClouds[k]->points.size();

        pF.centroid_3f = vec_centerG[k].block<3,1>(0,0);
        std::cout << "centroid : " << pF.centroid_3f << std::endl;

        pcl::ModelCoefficients temp = ransac_planeCoefficients[k];
        pF.planeCoef_4f << temp.values[0], temp.values[1], temp.values[2], temp.values[3];
        std::cout << "planeCoef_4f : " << pF.planeCoef_4f << std::endl;

        vec_planeFeature.push_back(pF);
    }
    vec_planes.push_back(vec_planeFeature);

    // visualize all planes obtained from RANSAC
    PointCloudPtr planesPtr(new PointCloudType);
    for(int i = 0; i < size_plane_clouds; i++)
    {
        *planesPtr += *(ransac_planeClouds[i]);
    }
    publishPlanesCloud(planesPtr);

    // visualize all planes centroid
    publishCentroidMarker(vec_centerG);

    clouds_.erase( clouds_.begin() ); // get the begin one, extract planes and then erase it
}

float Extr_PLANEFEATURE::calculate_normalErr(Eigen::Vector4f tarNor, Eigen::Vector4f sourNor)
{
    float dotProduct = tarNor(0)*sourNor(0) + tarNor(1)*sourNor(1) + tarNor(2)*sourNor(2);
    // double tarLen = sqrt(pow(tarNor(1),2.0) + pow(tarNor(2),2.0) + pow(tarNor(3),2.0));
    // double sourLen = sqrt(pow(sourNor(1),2.0) + pow(sourNor(2),2.0) + pow(sourNor(3),2.0));
    float cosErr = dotProduct / (1.0 * 1.0);
    return cosErr;
}

float Extr_PLANEFEATURE::calculate_centrErr(Eigen::Vector3f tarCen, Eigen::Vector3f sourCen)
{
    float cenErr = sqrt(pow((tarCen(0) - sourCen(0)),2.0) + 
        pow((tarCen(1)-sourCen(1)),2.0) + pow((tarCen(2)-sourCen(2)),2.0));

    return cenErr;
}

double Extr_PLANEFEATURE::calculate_sizeErr(int tarS, int sourS)
{
    double ts = tarS;
    double ss = sourS;
    double sizeErr = abs((ts-ss)/ts);
    return sizeErr;
}

void Extr_PLANEFEATURE::planeCorrespondence(Eigen::Matrix4f ndtResult)
{
    if(vec_planes.size() < 2)
    {
        std::cout << "No enough point cloud frame in vec_planes!" << std::endl;
        return;
    }

    // parameters setting
    int sizeThres = atoi(pr.getData("SizeThrese").c_str());
    float normalThres = atof(pr.getData("NormalThres").c_str());
    float centroidThres = atof(pr.getData("CentroidThres").c_str());

    std::vector<Plane_Feature> targetFeature;
    std::vector<Plane_Feature> sourceFeature;

    targetFeature = vec_planes[0];
    vec_planes.erase (vec_planes.begin());
    sourceFeature = vec_planes[0];

    int rowsize = targetFeature.size();
    int columnsize = sourceFeature.size();
    Eigen::MatrixXi corresMat_bool(rowsize, columnsize);
    corresMat_bool = Eigen::MatrixXi::Zero(rowsize, columnsize);
    Eigen::MatrixXf corresMat_value(rowsize, columnsize);
    corresMat_value = Eigen::MatrixXf::Zero(rowsize, columnsize);

    // record the biggest error of 3 parameters
    // used for calculating the Similarity function
    double max_normalErr = 0.0;
    double max_centroidErr = 0.0;
    double max_sizeErr = 0.0;
    float Wnor = 0.0, Wcen = 0.0, Wsize = 0.0;

    // for all planes in targetFeature
    for (int i = 0; i < rowsize; i++)
    {
        // for all planes in sourceFeature
        for(int j = 0; j < columnsize; j++)
        {
            float normalErr = calculate_normalErr(targetFeature[i].planeCoef_4f, sourceFeature[j].planeCoef_4f);
            if (normalErr < normalThres)
                break;

            float centroidErr = calculate_centrErr(targetFeature[i].centroid_3f, sourceFeature[j].centroid_3f);
            if (centroidErr > centroidThres)
                break;

            double sizeErr = calculate_sizeErr(targetFeature[i].size, sourceFeature[j].size);
            if (sizeErr > centroidThres)
                break;

            corresMat_bool(i,j) = 1;
        }
    }

    // check the correpondence Mat
    std::cout << "corresMat_bool : " << std::endl << corresMat_bool << std::endl;
    // std::cout << "corresMat_bool : " << std::endl << corresMat_bool << std::endl;
    // for (int i = 0; i < rowsize; i++)
    // {
    //     for (int j = 0; j < columnsize; j++)
    //     {
            
    //     }
    // }


    // Normal correspondence
    

    // centroid correspondence

    // size correspondence
}