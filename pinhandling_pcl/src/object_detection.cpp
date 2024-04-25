#include "object_detection.h"


objectDetectionNode::objectDetectionNode()
{
    m_cloudSubscriber = nh.subscribe("/rslidar_points", 1, &objectDetectionNode::pointCloudCallback, this);

    m_objectPublisher = nh.advertise<pinhandling_pcl::object_list>("/detection/objects", 1);
    m_markerPublisher = nh.advertise<visualization_msgs::MarkerArray>("/visual/objects", 1);
    m_pclDebugPublisher = nh.advertise<sensor_msgs::PointCloud2>("/debug/pcl/object_detection", 1);
}


void objectDetectionNode::pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
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
    pass.setFilterLimits (0.0, 20.0);
    pass.filter(*cloud);

    pass.setFilterFieldName("y");
    pass.setFilterLimits (-15.0, 15.0);
    pass.filter(*cloud);

    pass.setFilterFieldName("z");
    pass.setFilterLimits (-0.5, 5.0);
    pass.filter(*cloud);

    /**
     *  Conversion for segmentation
    */
    pcl::PointCloud<pcl::PointXYZ> *xyz_cloud = new pcl::PointCloud<pcl::PointXYZ>;
    pcl::PointCloud<pcl::PointXYZ>::Ptr xyzCloudPtr(xyz_cloud);
    pcl::fromPCLPointCloud2(*cloudPtr, *xyzCloudPtr);

    /**
    *   Setup search tree
    */
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(xyzCloudPtr);

    /**
    *   Object clustering
    */
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.1);
    ec.setMinClusterSize(20);
    ec.setMaxClusterSize(20000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(xyzCloudPtr);
    ec.extract(cluster_indices);

    pinhandling_pcl::object_list objectList;

    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
    {
        pcl::PointCloud<pcl::PointXYZ> *cluster = new pcl::PointCloud<pcl::PointXYZ>;
        pcl::PointCloud<pcl::PointXYZ>::Ptr clusterPtr(cluster);

        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
        {
            clusterPtr->points.push_back(xyzCloudPtr->points[*pit]);
        }

        /**
        *   Compute object data from each detected cluster
        */
        pinhandling_pcl::object_data object;
        Eigen::Matrix<double, 3, 1> centroid;
        Eigen::Matrix<double, 3, 1> obb_center;
        Eigen::Matrix<double, 3, 1> obb_dimensions;
        Eigen::Matrix<double, 3, 3> obb_rotational_matrix;

        objectDetectionNode::computeCentroidAndOBB(*clusterPtr, centroid, obb_center, obb_dimensions, obb_rotational_matrix);
        
        Eigen::Quaternion<double>  q(obb_rotational_matrix);
        q.normalize();

        object.centroid.x = centroid[0];
        object.centroid.y = centroid[1];
        object.centroid.z = centroid[2];
        object.obb_center.x = obb_center[0];
        object.obb_center.y = obb_center[1];
        object.obb_center.z = obb_center[2];
        object.obb_dimensions.x = obb_dimensions[0];
        object.obb_dimensions.y = obb_dimensions[1];
        object.obb_dimensions.z = obb_dimensions[2];        
        object.obb_rotation.x = q.x();
        object.obb_rotation.y = q.y();
        object.obb_rotation.z = q.z();
        object.obb_rotation.w = q.w();

        objectList.objects.push_back(object);
    }

    /**
    *   Publish data
    */
    m_objectPublisher.publish(objectList);
    objectDetectionNode::visualization(objectList);

    pcl::PCLPointCloud2 debugPCL;
    sensor_msgs::PointCloud2 debug;
    pcl::toPCLPointCloud2(*xyzCloudPtr, debugPCL);
    pcl_conversions::fromPCL(debugPCL, debug);
    m_pclDebugPublisher.publish(debug);
}


/**
*   Visualization of the calculated object list as a marker array
*   Each object is represented by its oriented boundary box (OBB)
*/
void objectDetectionNode::visualization(const pinhandling_pcl::object_list &objectList)
{
    visualization_msgs::MarkerArray markerArray;

    for (unsigned int i = 0; i < objectList.objects.size(); i++)
    {
        visualization_msgs::Marker marker;

        marker.header.frame_id = "rslidar";
        marker.id = i;
        marker.type = marker.CUBE;
        marker.action = marker.ADD;
        marker.pose.position = objectList.objects[i].obb_center;
        marker.pose.orientation = objectList.objects[i].obb_rotation;
        marker.scale = objectList.objects[i].obb_dimensions;
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;

        markerArray.markers.push_back(marker);
    }

    m_markerPublisher.publish(markerArray);
}


/**
*   Calculation of the oriented bounding box (OBB) of a give point cloud
*   Implementation of https://pointclouds.org/documentation/group__common.html#ga0ef093d77f87e8a3acb288fe6b8fa397
*/
unsigned int objectDetectionNode::computeCentroidAndOBB(const pcl::PointCloud<pcl::PointXYZ> &cloud,
                                    Eigen::Matrix<double, 3, 1> &centroid,
                                    Eigen::Matrix<double, 3, 1> &obb_center,
                                    Eigen::Matrix<double, 3, 1> &obb_dimensions,
                                    Eigen::Matrix<double, 3, 3> &obb_rotational_matrix)
{
    Eigen::Matrix<double, 3, 3> covariance_matrix;
    Eigen::Matrix<double, 4, 1> centroid4;
    const auto point_count = pcl::computeMeanAndCovarianceMatrix(cloud, covariance_matrix, centroid4);
    if (!point_count)
        return (0);
    centroid(0) = centroid4(0);
    centroid(1) = centroid4(1);
    centroid(2) = centroid4(2);

    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 3, 3>> evd(covariance_matrix);
    const Eigen::Matrix<double, 3, 3> eigenvectors_ = evd.eigenvectors();
    const Eigen::Matrix<double, 3, 1> minor_axis = eigenvectors_.col(0);
    const Eigen::Matrix<double, 3, 1> middle_axis = eigenvectors_.col(1);
    const Eigen::Matrix<double, 3, 1> major_axis = middle_axis.cross(minor_axis);

    obb_rotational_matrix <<
        major_axis(0), middle_axis(0), minor_axis(0),
        major_axis(1), middle_axis(1), minor_axis(1),
        major_axis(2), middle_axis(2), minor_axis(2);

    Eigen::Matrix<double, 4, 4> transform = Eigen::Matrix<double, 4, 4>::Identity();
    transform.topLeftCorner(3, 3) = obb_rotational_matrix.transpose();
    transform.topRightCorner(3, 1) =-transform.topLeftCorner(3, 3)*centroid;

    double obb_min_pointx, obb_min_pointy, obb_min_pointz;
    double obb_max_pointx, obb_max_pointy, obb_max_pointz;
    obb_min_pointx = obb_min_pointy = obb_min_pointz = std::numeric_limits<double>::max();
    obb_max_pointx = obb_max_pointy = obb_max_pointz = std::numeric_limits<double>::min();

    if (cloud.is_dense)
    {
        const auto& point = cloud[0];
        Eigen::Matrix<double, 4, 1> P0(static_cast<double>(point.x), static_cast<double>(point.y) , static_cast<double>(point.z), 1.0);
        Eigen::Matrix<double, 4, 1> P = transform * P0;
  
        obb_min_pointx = obb_max_pointx = P(0);
        obb_min_pointy = obb_max_pointy = P(1);
        obb_min_pointz = obb_max_pointz = P(2);
  
        for (size_t i=1; i<cloud.size();++i)
        {
            const auto&  point = cloud[i];
            Eigen::Matrix<double, 4, 1> P0(static_cast<double>(point.x), static_cast<double>(point.y) , static_cast<double>(point.z), 1.0);
            Eigen::Matrix<double, 4, 1> P = transform * P0;
  
            if (P(0) < obb_min_pointx)
                obb_min_pointx = P(0);
            else if (P(0) > obb_max_pointx)
                obb_max_pointx = P(0);
            if (P(1) < obb_min_pointy)
                obb_min_pointy = P(1);
            else if (P(1) > obb_max_pointy)
                obb_max_pointy = P(1);
            if (P(2) < obb_min_pointz)
                obb_min_pointz = P(2);
            else if (P(2) > obb_max_pointz)
                obb_max_pointz = P(2);
        }
    }
    else
    {
        size_t i = 0;
        for (; i < cloud.size(); ++i)
        {
            const auto&  point = cloud[i];
            if (!isFinite(point))
                continue;
            Eigen::Matrix<double, 4, 1> P0(static_cast<double>(point.x), static_cast<double>(point.y) , static_cast<double>(point.z), 1.0);
            Eigen::Matrix<double, 4, 1> P = transform * P0;
  
            obb_min_pointx = obb_max_pointx = P(0);
            obb_min_pointy = obb_max_pointy = P(1);
            obb_min_pointz = obb_max_pointz = P(2);
            ++i;
            break;
        }
  
        for (; i<cloud.size();++i)
        {
            const auto&  point = cloud[i];
            if (!isFinite(point))
                continue;
            Eigen::Matrix<double, 4, 1> P0(static_cast<double>(point.x), static_cast<double>(point.y) , static_cast<double>(point.z), 1.0);
            Eigen::Matrix<double, 4, 1> P = transform * P0;
  
            if (P(0) < obb_min_pointx)
                obb_min_pointx = P(0);
            else if (P(0) > obb_max_pointx)
                obb_max_pointx = P(0);
            if (P(1) < obb_min_pointy)
                obb_min_pointy = P(1);
            else if (P(1) > obb_max_pointy)
                obb_max_pointy = P(1);
            if (P(2) < obb_min_pointz)
                obb_min_pointz = P(2);
            else if (P(2) > obb_max_pointz)
                obb_max_pointz = P(2);
        }
  
    }
  
    const Eigen::Matrix<double, 3, 1>
        shift((obb_max_pointx + obb_min_pointx) / 2.0f,
            (obb_max_pointy + obb_min_pointy) / 2.0f,
            (obb_max_pointz + obb_min_pointz) / 2.0f);
  
    obb_dimensions(0) = obb_max_pointx - obb_min_pointx;
    obb_dimensions(1) = obb_max_pointy - obb_min_pointy;
    obb_dimensions(2) = obb_max_pointz - obb_min_pointz;
  
    obb_center = centroid + obb_rotational_matrix * shift;
  
    return (point_count);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "object_detection");
    objectDetectionNode objectDetectionNode;
    ros::spin();
    return 0;
}