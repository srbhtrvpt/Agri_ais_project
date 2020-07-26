#define PCL_NO_PRECOMPILE
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
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <tf2/LinearMath/Quaternion.h>
#include <std_msgs/Header.h>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <string>


struct XYZIRGBNormal
{
    PCL_ADD_POINT4D;
    PCL_ADD_RGB;
    PCL_ADD_NORMAL4D;
    float curvature;
    PCL_ADD_INTENSITY;               
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW     // make sure our new allocators are aligned
} EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment

POINT_CLOUD_REGISTER_POINT_STRUCT (XYZIRGBNormal,           // here we assume a XYZ (as fields)
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (uint32_t, rgb, rgb)
                                   (float, normal_x, normal_x)
                                   (float, normal_y, normal_y)
                                   (float, normal_z, normal_z)
                                   (float, curvature, curvature)
                                    (float, intensity , intensity)


)

ros::Publisher point_cloud_pub, point_cloud_pub1, point_cloud_pub2, vis_marker_pub, vis_marker_pub1;

typedef pcl::PointXYZI PointT;
typedef pcl::PointXYZINormal PointNCT;
typedef pcl::Normal PointNT;

typedef pcl::PointXYZRGB PointC;
typedef pcl::PointCloud<PointC> ColoredPointCloud;
typedef pcl::PointCloud<XYZIRGBNormal> PointCloudT;

typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointCloud<PointNT> PointCloudN;
typedef pcl::PointCloud<PointNCT> PointCloudNC;

/*
std::vector<int> neighborhood_plus(int cc, int rr, PointCloud::Ptr cloud_in);
std::vector<int> neighborhood_sq(int cc, int rr, PointCloud::Ptr cloud_in);
bool point_valid(int cc, int rr, PointCloud::Ptr cloud_in); */

visualization_msgs::Marker init_normal_marker();
visualization_msgs::Marker init_plane_marker(tf2::Quaternion marker_quat, pcl::ModelCoefficients::Ptr coeffs);
bool write_to_file(std::string filepath, std::string data);
tf2::Quaternion set_orientation(pcl::ModelCoefficients::Ptr coeffs);

int neighborhood_radius, ksearch_radius, normal_visualisation_scale;
bool save_flag;
double NormalDistanceWeight, DistanceThreshold;
float curvature_threshold;
std::string filepath;
tf::TransformListener *listener;


tf2::Quaternion set_orientation(pcl::ModelCoefficients::Ptr coeffs)
{
    tf2::Quaternion quat;
    tf2::Vector3 n_vec;
    n_vec.setX(coeffs->values[0]);
    n_vec.setY(coeffs->values[1]);
    n_vec.setZ(coeffs->values[2]);
    n_vec = n_vec.normalized();
    tf2Scalar pitch = tf2Asin(-n_vec.getY());
    tf2Scalar yaw = tf2Atan2(n_vec.getX(), n_vec.getZ());
    tf2Scalar roll = 0;
    quat.setEuler(yaw, pitch, roll);
    quat = quat.normalized();
    return quat;
}

/*

bool point_valid(int cc, int rr, PointCloud::Ptr cloud_in)
{
    int point_indice = (rr)*cloud_in->width + (cc);
    if ((point_indice <= cloud_in->points.size() - 1) && (point_indice >= 0))
    {
        if (isfinite(cloud_in->at(cc, rr).x) && isfinite(cloud_in->at(cc, rr).y) && isfinite(cloud_in->at(cc, rr).z))
        {
            return true;
        }
    }
    return false;
}

std::vector<int> neighborhood_sq(int cc, int rr, PointCloud::Ptr cloud_in)
{
    std::vector<int> result;
    for (int i = -neighborhood_radius; i <= neighborhood_radius; i++)
    {
        for (int j = -neighborhood_radius; j <= neighborhood_radius; j++)
        {
            if (point_valid(cc + j, rr + i, cloud_in))
            {
                result.push_back((rr + i) * cloud_in->width + (cc + j));
            }
        }
    }
    return result;
}

std::vector<int> neighborhood_plus(int cc, int rr, PointCloud::Ptr cloud_in)
{
    std::vector<int> result;
    for (int i = -neighborhood_radius; i <= neighborhood_radius; i++)
    {
        if (point_valid(cc, rr + i, cloud_in))
        {
            result.push_back((rr + i) * cloud_in->width + (cc));
        }
    }
    for (int j = -neighborhood_radius; j <= neighborhood_radius; j++)
    {
        if (point_valid(cc + j, rr, cloud_in))
        {
            result.push_back((rr)*cloud_in->width + (cc + j));
        }
    }
    return result;
}
 */

visualization_msgs::Marker init_normal_marker()
{
    visualization_msgs::Marker normal_marker;
    normal_marker.ns = "normal_markers";
    normal_marker.id = 0;
    normal_marker.type = visualization_msgs::Marker::LINE_LIST;
    normal_marker.action = visualization_msgs::Marker::ADD;
    normal_marker.color.a = 1.0;
    normal_marker.scale.x = 0.01;
    normal_marker.pose.orientation.w = 1.0;

    return normal_marker;
}

visualization_msgs::Marker init_plane_marker(tf2::Quaternion marker_quat, pcl::ModelCoefficients::Ptr coeffs)
{
    visualization_msgs::Marker plane_marker;
    plane_marker.header.frame_id = "base_footprint";
    plane_marker.ns = "plane_markers";
    plane_marker.id = 0;
    plane_marker.type = visualization_msgs::Marker::CYLINDER;
    plane_marker.action = visualization_msgs::Marker::ADD;
    plane_marker.scale.x = 6;
    plane_marker.scale.y = 6;
    plane_marker.scale.z = DistanceThreshold;
    plane_marker.color.a = 1.0;
    plane_marker.color.r = 0.0;
    plane_marker.color.g = 0.66;
    plane_marker.color.b = 0.0;
    plane_marker.pose.orientation.x = marker_quat.getX();
    plane_marker.pose.orientation.y = marker_quat.getY();
    plane_marker.pose.orientation.z = marker_quat.getZ();
    plane_marker.pose.orientation.w = marker_quat.getW();
    plane_marker.pose.position.x = 5;
    plane_marker.pose.position.y = 0;
    plane_marker.pose.position.z = -1.0 * (1.0 * coeffs->values[3] + plane_marker.pose.position.x * coeffs->values[0] + plane_marker.pose.position.y * coeffs->values[1]) / coeffs->values[2];
    return plane_marker;
}

bool write_to_file(std::string filep, std::string data)
{
    std::ofstream file;
    file.open(filep, std::ios::out | std::ios::app);
    if (file.fail())
    {
        throw std::ios_base::failure(std::strerror(errno));
        return false;
    }
    file.exceptions(file.exceptions() | std::ios::failbit | std::ifstream::badbit);
    file << data << std::endl;
    ROS_INFO_THROTTLE(1, "w");
    file.close();
    return true;
}

void callback(const sensor_msgs::PointCloud2ConstPtr &cloud_ros)
{

    PointCloud::Ptr cloud_in(new PointCloud), cloud_inliers(new PointCloud), cloud_outliers(new PointCloud);
    ColoredPointCloud::Ptr cloud_in_c(new ColoredPointCloud);
    PointCloudN::Ptr cloud_normals(new PointCloudN);
    // PointCloudNC::Ptr cloud_out(new PointCloudNC), cloud_out1(new PointCloudNC);
    PointCloudT::Ptr cloud_out(new PointCloudT);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    // PointNCT point;
    XYZIRGBNormal point;
    std_msgs::ColorRGBA color;
    geometry_msgs::Point p;
    int counter = 0;
    float curv_avg = 0.f;
    std::string filename;
    visualization_msgs::Marker normal_marker = init_normal_marker(), plane_marker;
    pcl::fromROSMsg(*cloud_ros, *cloud_in_c);
    pcl::fromROSMsg(*cloud_ros, *cloud_in);
    std::vector<int> map;
    cloud_in->is_dense = false;
    pcl::removeNaNFromPointCloud(*cloud_in, *cloud_in, map);

    //normal estimation
    pcl::NormalEstimation<PointT, PointNT> ne;
    ne.setInputCloud(cloud_in);
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
    ne.setSearchMethod(tree);
    // ne.setRadiusSearch(0.3);
    ne.setKSearch(ksearch_radius);
    ne.compute(*cloud_normals);

    for (int i = 0; i < cloud_normals->points.size(); i++)
    {
        if (isfinite(cloud_in->points[i].x) && isfinite(cloud_in->points[i].y) && isfinite(cloud_in->points[i].z) && isfinite(cloud_in->points[i].intensity) &&
            isfinite(cloud_normals->points[i].normal_z) && isfinite(cloud_normals->points[i].normal_y) && isfinite(cloud_in_c->points[i].rgb) &&
            isfinite(cloud_normals->points[i].normal_x) && isfinite(cloud_normals->points[i].curvature))
        {
            counter++;
            curv_avg += cloud_normals->points[i].curvature;
            if (save_flag)
            {
                filename = filepath + std::to_string(cloud_normals->header.stamp) + "_curvature.txt";
                if (!write_to_file(filename, std::to_string(cloud_normals->points[i].curvature)))
                {
                    ROS_ERROR("%s: error writing value %.9f", __func__, cloud_normals->points[i].curvature);
                }
                filename = filepath + std::to_string(cloud_in->header.stamp) + "_intensity.txt";
                if (!write_to_file(filename, std::to_string(cloud_in->points[i].intensity)))
                {
                    ROS_ERROR("%s: error writing value %.9f", __func__, cloud_in->points[i].intensity);
                }
                filename = filepath + std::to_string(cloud_in->header.stamp) + "_x.txt";
                if (!write_to_file(filename, std::to_string(cloud_in->points[i].x)))
                {
                    ROS_ERROR("%s: error writing value %.9f", __func__, cloud_in->points[i].x);
                }
                filename = filepath + std::to_string(cloud_in->header.stamp) + "_y.txt";
                if (!write_to_file(filename, std::to_string(cloud_in->points[i].y)))
                {
                    ROS_ERROR("%s: error writing value %.9f", __func__, cloud_in->points[i].y);
                }
            }

            //normal marker
            if (cloud_normals->points[i].curvature < curvature_threshold)
            {
                color.r = 1.0;
                color.g = 0.0;
                color.b = 0.0;
            }
            else
            {
                color.r = 0.0;
                color.g = 0.0;
                color.b = 1.0;
            }
            p.x = cloud_in->points[i].x;
            p.y = cloud_in->points[i].y;
            p.z = cloud_in->points[i].z;
            normal_marker.points.push_back(p);
            normal_marker.colors.push_back(color);
            p.z += cloud_normals->points[i].normal_z * normal_visualisation_scale * 0.001;
            p.y += cloud_normals->points[i].normal_y * normal_visualisation_scale * 0.001;
            p.x += cloud_normals->points[i].normal_x * normal_visualisation_scale * 0.001;
            normal_marker.points.push_back(p);
            normal_marker.colors.push_back(color);

            //pcl xyznci
            point.x = cloud_in->points[i].x;
            point.y = cloud_in->points[i].y;
            point.z = cloud_in->points[i].z;
            point.normal_x = cloud_normals->points[i].normal_x;
            point.normal_y = cloud_normals->points[i].normal_y;
            point.normal_z = cloud_normals->points[i].normal_z;
            point.intensity = cloud_in->points[i].intensity;
            point.curvature = cloud_normals->points[i].curvature;
            point.rgb = cloud_in_c->points[i].rgb;
            cloud_out->push_back(point);
        }
    }
    curv_avg = float(curv_avg / counter);

    //std::cout << "count" <<  counter << "size" << cloud_normals->points.size() <<  "points" << points.size() <<  "\n";
    //std::cout << "avg " << curv_avg  << std::endl;

    normal_marker.header.frame_id = cloud_ros->header.frame_id;
    normal_marker.header.stamp = cloud_ros->header.stamp;
    vis_marker_pub.publish(normal_marker);

    cloud_out->header = cloud_in->header;
    if (save_flag)
    {
        sensor_msgs::PointCloud2::Ptr cloud_out1;
        pcl::toROSMsg(*cloud_out, *cloud_out1);
        try
        {
            pcl_ros::transformPointCloud("base_footprint", *cloud_out1, *cloud_out1, *listener);
            pcl::io::savePCDFile(filepath + std::to_string(cloud_out->header.stamp) + ".pcd", *cloud_out1);
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
        }     
    }
    point_cloud_pub2.publish(cloud_out);

    // plane normal model
    pcl::SACSegmentationFromNormals<PointT, PointNT> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);
    seg.setNormalDistanceWeight(NormalDistanceWeight);
    seg.setDistanceThreshold(DistanceThreshold);

    // plane model
    /*
    coefficients->values.resize(4);
    pcl::SACSegmentation<PointT> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setDistanceThreshold(DistanceThreshold); */

    seg.setInputCloud(cloud_in);
    seg.setInputNormals(cloud_normals);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.segment(*inliers, *coefficients);

    plane_marker = init_plane_marker(set_orientation(coefficients), coefficients);
    plane_marker.header.stamp = cloud_ros->header.stamp;
    vis_marker_pub1.publish(plane_marker);

    // std::cout << "size"<<inliers->indices.size();
    if (inliers->indices.size() == 0)
    {
        PCL_ERROR("Could not estimate a planar model.\n");
    }
    //extract
    else
    {
        pcl::ExtractIndices<PointT> extract;
        extract.setInputCloud(cloud_in);
        extract.setIndices(inliers);
        extract.setNegative(false);     // Extract the inliers
        extract.filter(*cloud_inliers); // cloud_inliers contains the plane
        point_cloud_pub1.publish(cloud_inliers);

        extract.setNegative(true); // Extract the outliers
        extract.filter(*cloud_outliers);
        point_cloud_pub.publish(cloud_outliers);
    }
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "segmentation_node");
    ros::NodeHandle nh;
    ros::NodeHandle nhPriv("~");
    nhPriv.param("neighborhood_radius", neighborhood_radius, 2);
    nhPriv.param("ksearch_radius", ksearch_radius, 16);
    nhPriv.param("normal_visualisation_scale", normal_visualisation_scale, 30);
    nhPriv.param("curvature_threshold", curvature_threshold, 0.08f);
    nhPriv.param("Distance_Threshold", DistanceThreshold, 0.05);
    nhPriv.param("NormalDistanceWeight", NormalDistanceWeight, 0.02);
    nhPriv.param("save_flag", save_flag, false);
    nhPriv.param<std::string>("output_filepath", filepath, "/home/srbh/agrirobo_proj/with_pcls/data/");

    // ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/sensor/laser/vlp16/front/pointcloud_xyzi", 1, callback);
    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("input_cloud", 10, callback);
    vis_marker_pub = nh.advertise<visualization_msgs::Marker>("normal_marker", 1, true);
    vis_marker_pub1 = nh.advertise<visualization_msgs::Marker>("plane_marker", 1, true);
    point_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("output_cloud", 10, true);
    point_cloud_pub1 = nh.advertise<sensor_msgs::PointCloud2>("output_cloud1", 10, true);
    point_cloud_pub2 = nh.advertise<sensor_msgs::PointCloud2>("output_cloud2", 10, true);

    ros::spin();
}