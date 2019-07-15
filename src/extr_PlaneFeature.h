#ifndef EXTR_PLANEFEATURE_H_
#define EXTR_PLANEFEATURE_H_

#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "Parameter_Reader.h"
#include "my_point_cloud.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

// #include "glog/logging.h" // for inline void AddCloud()

class Plane_Feature {
public:
    Eigen::Vector4f planeCoef_4f;
    Eigen::Vector3f centroid_3f;
    int size;

    Plane_Feature() 
    {
        planeCoef_4f = Eigen::Vector4f::Zero();
        centroid_3f = Eigen::Vector3f::Zero();
        size = 0;
    };
    ~Plane_Feature() {};
};

class Extr_PLANEFEATURE {

public:
    ParameterReader pr;

    Plane_Feature planeFeature;
    std::vector<Plane_Feature> vec_planeFeature;
    std::vector<std::vector<Plane_Feature>> vec_planes;

    Extr_PLANEFEATURE(){};
    ~Extr_PLANEFEATURE(){};

    void Initialise( ros::NodeHandle& nh, std::string topic_name );

    inline void 
    AddCloud( const PointCloudPtr& cloud ) {
        if( cloud && !cloud->empty() ) {
            clouds_.push_back( *cloud );
        } else {
            std::cout << "AddCloud. nullptr or empty cloud" << std::endl;
        }
    }

    void run_extraction();

    void Extract_Indices(PointCloudPtr input_pc, PointCloudPtr output_pc,
                        PointIndices::Ptr input_indices, bool Neg_flag, bool Keep_flag = false);

    void Extract_from_indices(PointIndices &output, PointIndices inputIndices, PointIndices input);

    // bool sortIndices(const PointIndices& indice1 , const PointIndices& indice2);

    void Voxel_Grid();
    
    void Euclidean_Cluster_Extraction();

    void Region_Growing(std::vector<PointCloudPtr>& vec_segCloud,
                        std::vector<PointIndices>& vec_segIndices);

    void Ransac_Plane(std::vector<PointCloudPtr>& vec_clouds,
                    std::vector<pcl::ModelCoefficients>& vec_coefficients,
                    PointIndices& segIndices,
                    PointCloudType& EuclideanCloud);

    void publishEuclideanCloud(PointCloudPtr inPtr);
    void publishSegCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inPtr);
    void publishPlanesCloud(PointCloudPtr inPtr);
    void publishCentroidMarker(std::vector<Eigen::Vector4f> inCentroidVec);

    float calculate_normalErr(Eigen::Vector4f tarNor, Eigen::Vector4f sourNor);

    float calculate_centrErr(Eigen::Vector3f tarCen, Eigen::Vector3f sourCen);

    double calculate_sizeErr(int tarS, int sourS);

    void planeCorrespondence(Eigen::Matrix4f ndtResult);

protected:
    void Callback(const sensor_msgs::PointCloud2::ConstPtr& msg);

private:
    ros::Subscriber subscriber_;
    ros::Publisher pubProcessCloud_A;
    ros::Publisher pubProcessCloud_B;
    ros::Publisher pubProcessCloud_C;
    ros::Publisher pubCentroidMarker;
    int laserCount;
    int sampleNum;
    bool got_first_data_;

    PointCloudPtr inPointCloud_ptr;
    PointCloudPtr outPointCloud_ptr;

    std::vector<PointCloudType> clouds_;

};

#endif