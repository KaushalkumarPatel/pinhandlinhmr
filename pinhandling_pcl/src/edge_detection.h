#ifndef EDGE_DETECTION_NODE
#define EDGE_DETECTION_NODE


#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/pcl_config.h>
#include <pcl/common/centroid.h>
#include <pcl/common/distances.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

#include <pinhandling_pcl/point_lists.h>


class edgeDetectionNode
{
    public:
        edgeDetectionNode();

    private:
        ros::NodeHandle nh;

        ros::Subscriber m_cloudSubscriber{};

        ros::Publisher m_edgePublisher{};
        ros::Publisher m_pclDebugPublisher{};

        void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg);

        Eigen::Matrix<double, 4, 1> m_prevCentroid{0.0, 0.0, 0.0, 0.0};
};


#endif