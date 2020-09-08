#include <tf2_ros/transform_listener.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <boost/bind.hpp>
#include "myproject1/pcl_coloriser.h" 

using namespace message_filters;
ros::Publisher point_cloud_pub;
ros::Subscriber sub;
std::string filepath;
bool save_flag;
PclColoriser* pcl_coloriser;
sensor_msgs::CameraInfo cam_info_msg;


void callback(const sensor_msgs::ImageConstPtr &image, const sensor_msgs::PointCloud2ConstPtr &cloud)
{
    if(!pcl_coloriser->setCameraInfo(cam_info_msg)){
        return;
    }
   sensor_msgs::PointCloud2::Ptr colorised_pcl = pcl_coloriser->colorisedCloud(image, cloud);
    point_cloud_pub.publish(*colorised_pcl);
    
}

void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr &info_msg)
{
    cam_info_msg = *info_msg;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "pcl_coloriser_node");
    ros::NodeHandle nh;
    ros::NodeHandle nhPriv("~");
    nhPriv.param<std::string>("op_path", filepath, "/home/srbh/agrirobo_proj/with_pcls/data/colored/");
    nhPriv.param("save_flag", save_flag, false);

    tf2_ros::Buffer tf_buffer;
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