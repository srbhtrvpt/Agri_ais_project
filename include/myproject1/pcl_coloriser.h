#ifndef _PCL_COLORISER_H_
#define _PCL_COLORISER_H_

#include <tf2_ros/buffer.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>


class PclColoriser
{
public:
    PclColoriser(std::string fixed_frame, tf2_ros::Buffer *tf_buffer, int max_buffer_size = 10);

    // bool integrate(const sensor_msgs::PointCloud2::ConstPtr &pcl_msg);
    sensor_msgs::PointCloud2::Ptr colorisedCloud(const sensor_msgs::ImageConstPtr &image, const sensor_msgs::PointCloud2ConstPtr &pcl_msg) const;

protected:
    int max_buffer_size_;
    std::string fixed_frame_;
    tf2_ros::Buffer *tf_buffer_;
    std::deque<sensor_msgs::PointCloud2::Ptr> cloud_buffer_;

    // helper functions
    sensor_msgs::PointCloud2::Ptr crop_pcl(const sensor_msgs::PointCloud2::Ptr &pcl) const;
    // implement transform point cloud
};

#endif // _PCL_COLORISER_H_
