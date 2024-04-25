#include "edge_detection.h"


edgeDetectionNode::edgeDetectionNode()
{
    m_cloudSubscriber = nh.subscribe("/rslidar_points", 1, &edgeDetectionNode::pointCloudCallback, this);

    m_edgePublisher = nh.advertise<pinhandling_pcl::point_lists>("/detection/edge", 1);
    m_pclDebugPublisher = nh.advertise<sensor_msgs::PointCloud2>("/debug/pcl/edge_detection", 1);
}


void edgeDetectionNode::pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    /**
     *  Definition of PCL data container and conversion
    */
    pcl::PCLPointCloud2 *cloud = new pcl::PCLPointCloud2; 
    pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
    pcl_conversions::toPCL(*msg, *cloud);

    /**
    *   Filter Region-of-Interest
    */
    pcl::PassThrough<pcl::PCLPointCloud2> pass;
    pass.setInputCloud(cloudPtr);
    pass.setFilterFieldName("x");
    pass.setFilterLimits (0.0, 10.0);
    pass.filter(*cloud);

    pass.setFilterFieldName("y");
    pass.setFilterLimits (-2.5, 2.5);
    pass.filter(*cloud);

    pass.setFilterFieldName("z");
    pass.setFilterLimits (-1.5, -0.5);
    pass.filter(*cloud);

    /**
    *  Conversion for segmentation
    */
    pcl::PointCloud<pcl::PointXYZ> *xyz_cloud = new pcl::PointCloud<pcl::PointXYZ>;
    pcl::PointCloud<pcl::PointXYZ>::Ptr xyzCloudPtr(xyz_cloud);
    pcl::fromPCLPointCloud2(*cloudPtr, *xyzCloudPtr);

    /**
    *  Plane Segmentation
    */
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.02);
    seg.setInputCloud(xyzCloudPtr);
    seg.segment(*inliers, *coefficients);

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(xyzCloudPtr);
    extract.setIndices(inliers);
    extract.filter(*xyzCloudPtr);

    /**
    *   Removing outlier in the same plane
    */
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(xyzCloudPtr);
    sor.setMeanK(10);
    sor.setStddevMulThresh(1.0);
    sor.filter(*xyzCloudPtr);

    /**
    *   Calculate the mean of the filtered point cloud and compare it with the data of the previous iteration
    *   Give a warning if a different plane was segmented in the current callback (or if the data is otherwise shifted too much)
    * 
    *   eps:= Maximum allowed difference in [m] (for each coordinate) between two point clouds
    * 
    *   TODO: Currently no action taken w.r.t. the data processing when a deviation is detected
    */
    const double eps = 0.5;
    Eigen::Matrix<double, 4, 1> centroid;
    pcl::compute3DCentroid(*xyzCloudPtr, centroid);
    if (centroid[0] - m_prevCentroid[0] > eps || centroid[1] - m_prevCentroid[1] > eps || centroid[2] - m_prevCentroid[2] > eps)
    {
        ROS_WARN("DEVIATING GROUND PLANE DETECTED! [dX=%f, dY=%f, dZ=%f]", centroid[0]-m_prevCentroid[0], centroid[1]-m_prevCentroid[1], centroid[2]-m_prevCentroid[2]);
    }
    m_prevCentroid = centroid;

    /**
    *   Setup search tree
    */
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(xyzCloudPtr);

    /**
    *   Clustering the individual scan lines
    */
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.08);
    ec.setMinClusterSize(150);
    ec.setMaxClusterSize(1000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(xyzCloudPtr);
    ec.extract(cluster_indices);

    pinhandling_pcl::point_lists edgePoints; 

    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
    {
        pcl::PointCloud<pcl::PointXYZ> *cluster = new pcl::PointCloud<pcl::PointXYZ>;
        pcl::PointCloud<pcl::PointXYZ>::Ptr clusterPtr(cluster);

        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
        {
            clusterPtr->points.push_back(xyzCloudPtr->points[*pit]);
        }

        /**
        *   Compute the outer points for each cluster
        */
        pcl::PointXYZ pmin, pmax;
        geometry_msgs::Point point_min, point_max;
        pcl::getMinMax3D(*clusterPtr, pmin, pmax);

        point_min.x = pmin.x;
        point_min.y = pmin.y;
        point_min.z = pmin.z;

        point_max.x = pmax.x;
        point_max.y = pmax.y;
        point_max.z = pmax.z;

        edgePoints.points_min.push_back(point_min);
        edgePoints.points_max.push_back(point_max);

        /**
        *   Compute the middle lane via the centroid of each respective cluster instead
        */
        // geometry_msgs::Point point_mid
        // Eigen::Matrix<double, 4, 1> centroid2;
        // pcl::compute3DCentroid(*clusterPtr, centroid2);
        // point_mid.x = centroid2[0];
        // point_mid.y = centroid2[1];
        // point_mid.z = centroid2[2];
        // // edgePoints.points_min.push_back(point_mid);
    }

    /**
    *   Publish data
    */
    m_edgePublisher.publish(edgePoints);

    pcl::PCLPointCloud2 debugPCL;
    sensor_msgs::PointCloud2 debug;
    pcl::toPCLPointCloud2(*xyzCloudPtr, debugPCL);
    pcl_conversions::fromPCL(debugPCL, debug);
    m_pclDebugPublisher.publish(debug);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "edge_detection");
    edgeDetectionNode edgeDetectionNode;
    ros::spin();
    return 0;
}