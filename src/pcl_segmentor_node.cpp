#include "myproject1/pcl_segmentor.h"
#include <pcl_ros/transforms.h>
#include <visualization_msgs/Marker.h>
#include <tf2/LinearMath/Quaternion.h>
#include <std_msgs/Header.h>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <string>
#include <stdio.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <boost/bind.hpp>

using namespace message_filters;
/*
std::vector<int> neighborhood_plus(int cc, int rr, PointCloud::Ptr cloud_in);
std::vector<int> neighborhood_sq(int cc, int rr, PointCloud::Ptr cloud_in);
bool point_valid(int cc, int rr, PointCloud::Ptr cloud_in); */

// visualization_msgs::Marker init_normal_marker();
visualization_msgs::Marker init_plane_marker(tf2::Quaternion marker_quat, pcl::ModelCoefficients::Ptr coeffs);
geometry_msgs::Quaternion set_orientation(pcl::ModelCoefficients::Ptr coeffs);

int neighborhood_radius, ksearch_radius, normal_visualisation_scale;
bool save_flag;
double NormalDistanceWeight, DistanceThreshold;
float curvature_threshold;
geometry_msgs::TransformStamped transformStamped;
ros::Publisher point_cloud_pub3, point_cloud_pub1, point_cloud_pub2, vis_marker_pub, vis_marker_pub1;
PclSegmentor* pcl_segmentor;
std::string filepath;
tf2_ros::Buffer tf_buffer;

geometry_msgs::Quaternion set_orientation(pcl::ModelCoefficients::Ptr coeffs){
    tf2::Quaternion quat;
    tf2::Vector3 n_vec;

    n_vec.setX(coeffs->values[0]);
    n_vec.setY(coeffs->values[1]);
    n_vec.setZ(coeffs->values[2]);
    // n_vec = n_vec.normalized();
    
    double pitch, yaw, roll;
    pitch = tf2Asin(-n_vec.getY());
    yaw = tf2Atan2(n_vec.getX(), n_vec.getZ());
    roll = 0;
    quat.setRPY(pitch,yaw , roll);
    quat = quat.normalized();

    geometry_msgs::Quaternion orientation;
    orientation = tf2::toMsg(quat);
    return orientation;
}

// visualization_msgs::Marker init_normal_marker(){

//     visualization_msgs::Marker normal_marker;
//     normal_marker.ns = "normal_markers";
//     normal_marker.id = 0;
//     normal_marker.type = visualization_msgs::Marker::LINE_LIST;
//     normal_marker.action = visualization_msgs::Marker::ADD;
//     normal_marker.color.a = 1.0;
//     normal_marker.scale.x = 0.01;
//     normal_marker.pose.orientation.w = 1.0;

//     return normal_marker;
// }

visualization_msgs::Marker init_plane_marker(geometry_msgs::Quaternion marker_quat, pcl::ModelCoefficients::Ptr coeffs){

    visualization_msgs::Marker plane_marker;
    plane_marker.header.frame_id = "base_footprint";
    plane_marker.ns = "plane_markers";
    plane_marker.id = 0;
    plane_marker.type = visualization_msgs::Marker::CYLINDER;
    plane_marker.action = visualization_msgs::Marker::ADD;
    plane_marker.scale.x = 3;
    plane_marker.scale.y = 3;
    plane_marker.scale.z = DistanceThreshold;
    plane_marker.color.a = 1.0;
    plane_marker.color.r = 0.0;
    plane_marker.color.g = 0.66;
    plane_marker.color.b = 0.0;
    plane_marker.pose.orientation.x = marker_quat.x;
    plane_marker.pose.orientation.y = marker_quat.y;
    plane_marker.pose.orientation.z = marker_quat.z;
    plane_marker.pose.orientation.w = marker_quat.w;
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

void callback(const sensor_msgs::PointCloud2ConstPtr &cloud_ros2, const sensor_msgs::PointCloud2ConstPtr &cloud_ros){

    sensor_msgs::PointCloud2::Ptr pcl_msg(new sensor_msgs::PointCloud2()), pcl_plants(new sensor_msgs::PointCloud2());
    *pcl_msg = *cloud_ros;
    *pcl_plants = *cloud_ros2;
    try{
        transformStamped = tf_buffer.lookupTransform("base_footprint",pcl_msg->header.frame_id, pcl_msg->header.stamp);
    }
    catch (tf2::TransformException &ex){
        ROS_ERROR("%s", ex.what());       
    }
    tf2::doTransform(*pcl_msg, *pcl_msg, transformStamped);
    pcl_segmentor = new PclSegmentor(pcl_msg);
    if(!pcl_segmentor->computePclNormals(ksearch_radius)){
        return;
    }

    if(!pcl_segmentor->segmentPcl(NormalDistanceWeight,DistanceThreshold)){
        return;
    }
    pcl::ModelCoefficients::Ptr coefficients = pcl_segmentor->getCoefficients();
    visualization_msgs::Marker plane_marker = init_plane_marker(set_orientation(coefficients), coefficients);
    plane_marker.header.stamp = cloud_ros->header.stamp;
    vis_marker_pub1.publish(plane_marker);

    sensor_msgs::PointCloud2::Ptr cloud_inliers = pcl_segmentor->inlierCloud();
    try{
        transformStamped = tf_buffer.lookupTransform(pcl_plants->header.frame_id,"base_footprint", pcl_msg->header.stamp);
        tf2::doTransform(*cloud_inliers, *cloud_inliers, transformStamped);
    }
    catch (tf2::TransformException &ex){
        ROS_ERROR("%s", ex.what());       
    }
    point_cloud_pub2.publish(*cloud_inliers);

    sensor_msgs::PointCloud2::Ptr cloud_outliers = pcl_segmentor->outlierCloud();
    try{
        transformStamped = tf_buffer.lookupTransform(pcl_plants->header.frame_id,"base_footprint", pcl_msg->header.stamp);
        tf2::doTransform(*cloud_outliers, *cloud_outliers, transformStamped);
    }
    catch (tf2::TransformException &ex){
        ROS_ERROR("%s", ex.what());       
    }
    point_cloud_pub3.publish(*cloud_outliers);

    PclSegmentor *pcl_segmentor2 = new PclSegmentor(pcl_plants);
    if(!pcl_segmentor2->computePclNormals(ksearch_radius)){
        return;
    }
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs_cloud = pcl_segmentor2->fpfhs();
    
    point_cloud_pub1.publish(*fpfhs_cloud);    
    // normal_marker.header.frame_id = cloud_ros->header.frame_id;
    // normal_marker.header.stamp = cloud_ros->header.stamp;
    // vis_marker_pub.publish(normal_marker);


    // std::string filename;
    // try{
    //     transformStamped = tf_buffer.lookupTransform("base_footprint",pcl_plants->header.frame_id, pcl_plants->header.stamp);
    // }
    // catch (tf2::TransformException &ex){
    //     ROS_ERROR("%s", ex.what());       
    // }
    // tf2::doTransform(*pcl_plants, *pcl_plants, transformStamped);

    // sensor_msgs::PointCloud2Iterator<float> out_x(*pcl_plants, "x");
    // sensor_msgs::PointCloud2Iterator<float> out_y(*pcl_plants, "y");
    // sensor_msgs::PointCloud2Iterator<float> out_z(*pcl_plants, "z");
    // sensor_msgs::PointCloud2Iterator<float> out_int(*pcl_plants, "intensity");
    // sensor_msgs::PointCloud2Iterator<float> out_rgb(*pcl_plants, "rgb");
    // sensor_msgs::PointCloud2Iterator<float> out_tgi(*pcl_plants, "tgi");
    // sensor_msgs::PointCloud2Iterator<float> out_vari(*pcl_plants, "vari");
    // sensor_msgs::PointCloud2Iterator<float> out_curv(*pcl_plants, "curvature");
        
    // filename = filepath + std::to_string(pcl_plants->header.stamp.toNSec()) + ".txt";
    // while (out_x != out_x.end()){ 
    //     geometry_msgs::Point p; 
    //     p.x = *out_x;
    //     p.y = *out_y;
    //     p.z = *out_z;
    //     double alt = pcl::pointToPlaneDistanceSigned(p, coefficients->values[0], coefficients->values[1], coefficients->values[2],coefficients->values[3]);
    //     if (!write_to_file(filename, std::to_string(*out_x) 
    //                                 + "\t" + std::to_string(*out_y) 
    //                                 + "\t" + std::to_string(*out_z) 
    //                                 + "\t" + std::to_string(*out_int) 
    //                                 + "\t" + std::to_string(*out_rgb)  
    //                                 + "\t" + std::to_string(*out_tgi)  
    //                                 + "\t" + std::to_string(*out_vari)  
    //                                 + "\t" + std::to_string(*out_curv)//)){
    //                                 + "\t" + std::to_string(alt))){
    //         ROS_ERROR("%s: error writing value ", __func__);
    //     }
    //     ++out_x;
    //     ++out_y;
    //     ++out_z;
    //     ++out_int;
    //     ++out_rgb;
    //     ++out_tgi;
    //     ++out_curv;
    //     ++out_vari;
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
    // nhPriv.param("curvature_threshold", curvature_threshold, 0.08f);
    nhPriv.param("Distance_Threshold", DistanceThreshold, 0.05);
    nhPriv.param("NormalDistanceWeight", NormalDistanceWeight, 0.02);

    // nhPriv.param("save_", save_flag, false);
    nhPriv.param<std::string>("output_fpath", filepath, "/home/srbh/agrirobo_proj/with_pcls/data/full_pcl/");

    tf2_ros::TransformListener tf_listener(tf_buffer);
    message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub(nh, "input_cloud", 1);
    message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub2(nh, "input_cloud2", 1);
    typedef sync_policies::ExactTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> MySyncPolicy;

    Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), cloud_sub2, cloud_sub);
    sync.registerCallback(boost::bind(&callback, _1, _2));
    // ros::Subscriber sub1 = nh.subscribe<sensor_msgs::PointCloud2>("input_cloud2", 10, callback);
    // ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("input_cloud", 10, callback);
    // vis_marker_pub = nh.advertise<visualization_msgs::Marker>("normal_marker", 1, true);
    vis_marker_pub1 = nh.advertise<visualization_msgs::Marker>("plane_marker", 1, true);
    point_cloud_pub1 = nh.advertise<sensor_msgs::PointCloud2>("output_cloud1", 10, true);
    point_cloud_pub2 = nh.advertise<sensor_msgs::PointCloud2>("output_cloud2", 10, true);
    point_cloud_pub3 = nh.advertise<sensor_msgs::PointCloud2>("output_cloud3", 10, true);

    ros::spin();
}