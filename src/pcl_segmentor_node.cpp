#include "myproject1/pcl_segmentor.h"
#include <pcl_ros/transforms.h>
#include <visualization_msgs/Marker.h>
#include <tf2/LinearMath/Quaternion.h>
#include <std_msgs/Header.h>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <string>

ros::Publisher point_cloud_pub, point_cloud_pub1, point_cloud_pub2, vis_marker_pub, vis_marker_pub1;
PclSegmentor* pcl_segmentor;

typedef pcl::PointXYZI PointT;
typedef pcl::Normal PointNT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointCloud<PointNT> PointCloudN;

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

visualization_msgs::Marker init_normal_marker(){

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

visualization_msgs::Marker init_plane_marker(tf2::Quaternion marker_quat, pcl::ModelCoefficients::Ptr coeffs){

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

bool write_to_file(std::string filep, std::string data){

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

void callback(const sensor_msgs::PointCloud2ConstPtr &cloud_ros){

    pcl_segmentor = new PclSegmentor(cloud_ros);
    sensor_msgs::PointCloud2::Ptr cloud_out = pcl_segmentor->curvatureCloud(ksearch_radius);
    point_cloud_pub1.publish(*cloud_out);
    if(!pcl_segmentor->segmentPcl(NormalDistanceWeight,DistanceThreshold)){
        return;
    }
    sensor_msgs::PointCloud2::Ptr cloud_inliers = pcl_segmentor->segmentedCloud();
    point_cloud_pub2.publish(*cloud_inliers);
    
    pcl::ModelCoefficients::Ptr coefficients = pcl_segmentor->getCoefficients();
    visualization_msgs::Marker plane_marker = init_plane_marker(set_orientation(coefficients), coefficients);
    plane_marker.header.stamp = cloud_ros->header.stamp;
    vis_marker_pub1.publish(plane_marker);
    // normal_marker.header.frame_id = cloud_ros->header.frame_id;
    // normal_marker.header.stamp = cloud_ros->header.stamp;
    // vis_marker_pub.publish(normal_marker);
    // std::string filename;
    // if (save_flag)
    // {
    //     sensor_msgs::PointCloud2::Ptr cloud_out1;
    //     pcl::toROSMsg(*cloud_out, *cloud_out1);
    //     try
    //     {
    //         pcl_ros::transformPointCloud("base_footprint", *cloud_out1, *cloud_out1, *listener);
    //         pcl::io::savePCDFile(filepath + std::to_string(cloud_out->header.stamp) + ".pcd", *cloud_out1);
    //     }
    //     catch(const std::exception& e)
    //     {
    //         std::cerr << e.what() << '\n';
    //     }     
    // }
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "pcl_segmentor_node");
    ros::NodeHandle nh;
    ros::NodeHandle nhPriv("~");
    // nhPriv.param("neighborhood_radius", neighborhood_radius, 2);
    nhPriv.param("ksearch_radius", ksearch_radius, 16);
    // nhPriv.param("normal_visualisation_scale", normal_visualisation_scale, 30);
    nhPriv.param("curvature_threshold", curvature_threshold, 0.08f);
    nhPriv.param("Distance_Threshold", DistanceThreshold, 0.05);
    nhPriv.param("NormalDistanceWeight", NormalDistanceWeight, 0.02);
    nhPriv.param("save_flag", save_flag, false);
    nhPriv.param<std::string>("output_filepath", filepath, "/home/srbh/agrirobo_proj/with_pcls/data/");

    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("input_cloud", 10, callback);
    // vis_marker_pub = nh.advertise<visualization_msgs::Marker>("normal_marker", 1, true);
    vis_marker_pub1 = nh.advertise<visualization_msgs::Marker>("plane_marker", 1, true);
    // point_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("output_cloud", 10, true);
    point_cloud_pub1 = nh.advertise<sensor_msgs::PointCloud2>("output_cloud1", 10, true);
    point_cloud_pub2 = nh.advertise<sensor_msgs::PointCloud2>("output_cloud2", 10, true);

    ros::spin();
}