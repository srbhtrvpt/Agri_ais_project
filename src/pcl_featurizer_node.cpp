#include <tf2_ros/transform_listener.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <boost/bind.hpp>
// #include <rosbag/bag.h>
#include <fstream>
#include "myproject1/pcl_coloriser.h" 
#include "myproject1/pcl_segmentor.h"
#include <stdlib.h>

using namespace message_filters;
ros::Publisher point_cloud_pub;
ros::Subscriber sub;
PclColoriser* pcl_coloriser;
sensor_msgs::CameraInfo cam_info_msg;
PclSegmentor* pcl_segmentor;
int ksearch_radius;
tf2_ros::Buffer tf_buffer;
// std::string filepath;

void callback(const sensor_msgs::ImageConstPtr &image, const sensor_msgs::PointCloud2ConstPtr &cloud){

    if(!pcl_coloriser->setCameraInfo(cam_info_msg)){
        ROS_ERROR("error setting cam model");
        return;
    }
    sensor_msgs::PointCloud2::Ptr colorised_pcl = pcl_coloriser->colorisedCloud(image, cloud);
    pcl_segmentor = new PclSegmentor(colorised_pcl);
    if(!pcl_segmentor->computePclNormals(ksearch_radius)){
        ROS_ERROR("error computing normals");
        return;
    }
    sensor_msgs::PointCloud2::Ptr cloud_curv = pcl_segmentor->curvatureCloud();
    point_cloud_pub.publish(*cloud_curv);  
}

void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr &info_msg)
{
    cam_info_msg = *info_msg;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "pcl_featurizer_node");
    ros::NodeHandle nh;
    ros::NodeHandle nhPriv("~");
    nhPriv.param("KSearchRadius", ksearch_radius, 16);
    // nhPriv.param<std::string>("op_path", filepath, "/home/srbh/agrirobo_proj/with_pcls/data/");

    tf2_ros::TransformListener tf_listener(tf_buffer);
    sub = nh.subscribe("camera_info", 5, cameraInfoCallback);
    pcl_coloriser = new PclColoriser(&tf_buffer);

    message_filters::Subscriber<sensor_msgs::Image> image_sub(nh, "input_image", 10);
    message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub(nh, "input_cloud", 10);

    typedef sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2> MySyncPolicy;

    Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), image_sub, cloud_sub);
    sync.registerCallback(boost::bind(&callback, _1, _2));

    point_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("output_cloud", 10, true);

    ros::spin();
    return 0;
}