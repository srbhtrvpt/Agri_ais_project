#ifndef _PCL_SEGMENTOR_H_
#define _PCL_SEGMENTOR_H_

#include <pcl/ModelCoefficients.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/filter.h>
#include <pcl/io/pcd_io.h>
#include <pcl/pcl_config.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/pcl_macros.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

typedef pcl::PointXYZ PointT;
typedef pcl::Normal PointNT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointCloud<PointNT> PointCloudN;
class PclSegmentor
{
public:

    PclSegmentor(sensor_msgs::PointCloud2::Ptr pcl_msg_orig);
    sensor_msgs::PointCloud2::Ptr curvatureCloud();
    bool segmentPcl(double NormalDistanceWeight, double DistanceThreshold);
    pcl::PointIndices::Ptr getInlierIndices();
    pcl::ModelCoefficients::Ptr getCoefficients();
    PointCloudN::Ptr getCloudNormals();
    sensor_msgs::PointCloud2::Ptr inlierCloud();
    sensor_msgs::PointCloud2::Ptr outlierCloud();
    bool computePclNormals(int KSearchRadius);


protected:
    bool segmented, computed_normals;
    sensor_msgs::PointCloud2::Ptr pcl_msg;
    PointCloud::Ptr cloud_in{new PointCloud};
    PointCloudN::Ptr cloud_normals{new PointCloudN};
    pcl::PointIndices::Ptr inliers{new pcl::PointIndices};
    pcl::ModelCoefficients::Ptr coefficients{new pcl::ModelCoefficients};
    pcl::ExtractIndices<PointT> extract;

};

#endif // _PCL_SEGMENTOR_H_
