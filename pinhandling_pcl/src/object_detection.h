#ifndef OBJECT_DETECTION_NODE
#define OBJECT_DETECTION_NODE


#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/centroid.h>

#include <pinhandling_pcl/object_data.h>
#include <pinhandling_pcl/object_list.h>


class objectDetectionNode
{
    public:
        objectDetectionNode();

    private:
        ros::NodeHandle nh;

        ros::Subscriber m_cloudSubscriber{};

        ros::Publisher m_objectPublisher{};
        ros::Publisher m_markerPublisher{};
        ros::Publisher m_pclDebugPublisher{};

        void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg);

        void visualization(const pinhandling_pcl::object_list &objectList);

        unsigned int computeCentroidAndOBB (const pcl::PointCloud<pcl::PointXYZ> &cloud,
                                            Eigen::Matrix<double, 3, 1> &centroid,
                                            Eigen::Matrix<double, 3, 1> &obb_center,
                                            Eigen::Matrix<double, 3, 1> &obb_dimensions,
                                            Eigen::Matrix<double, 3, 3> &obb_rotational_matrix);
};


#endif