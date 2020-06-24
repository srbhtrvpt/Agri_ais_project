#include <tf2_ros/transform_listener.h>

#include "myproject1/pcl_integrator.h"

bool crop_flag;
std::string target_frame;

PclIntegrator* pcl_integrator;
ros::Publisher pcl_publisher;

void callback(sensor_msgs::PointCloud2::Ptr pcl_msg);

int main(int argc, char *argv[])
{
    
    ros::init(argc, argv, "pcl_integrator_node");

    ros::NodeHandle nh;
    ros::NodeHandle nhPriv("~");
    
    int max_buffer_size;
    float size_x, size_y, offset_x, offset_y;
    std::string fixed_frame, base_footprint, target_frame;
    ros::Publisher point_cloud_pub;

    // read paramters from rosparam server
    if (!nhPriv.getParam("max_buffer_size", max_buffer_size))
    {
        ROS_ERROR("max_buffer_size was not set!");
        return 1;
    }

    nhPriv.param("offset_y", offset_y, 0.f);
    nhPriv.param("offset_x", offset_x, 1.f);
    nhPriv.param("size_y", size_y, 6.f);
    nhPriv.param("size_x", size_x, 8.f);
    nhPriv.param("crop_flag", crop_flag, true);

    nhPriv.param<std::string>("fixed_frame", fixed_frame, "odom");
    nhPriv.param<std::string>("base_footprint", base_footprint, "base_footprint");
    target_frame = base_footprint;

    // initialize ros
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener(tf_buffer); // this will subscribe to tf and fill the tf_buffer
    
    Window window(base_footprint, size_x, size_y, offset_x, offset_y);

    pcl_integrator = new PclIntegrator(fixed_frame, window, &tf_buffer, max_buffer_size);
    
    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2::Ptr>("input_cloud", 10, callback);
    pcl_publisher = nh.advertise<sensor_msgs::PointCloud2>("output_cloud", 1, true);
    
    ros::spin();
    return 0;
}

void callback(sensor_msgs::PointCloud2::Ptr pcl_msg)
{
    if(!pcl_integrator->integrate(pcl_msg)){
//        ROS_ERROR("%s: error integrating pcl at time %.9f", __func__, pcl_msg->header.stamp.toSec());
        return;
    }
    sensor_msgs::PointCloud2::Ptr integrated_pcl = pcl_integrator->integratedCloud(target_frame, crop_flag);
    pcl_publisher.publish(*integrated_pcl);
}
