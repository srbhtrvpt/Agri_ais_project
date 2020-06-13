#ifndef _PCL_INTEGRATOR_H_
#define _PCL_INTEGRATOR_H_

#include <tf2_ros/buffer.h>

#include <sensor_msgs/PointCloud2.h>

#include "window.h"

class PclIntegrator
{
public:

    PclIntegrator(std::string fixed_frame, tf2::BufferCore* tf_buffer, int max_buffer_size=10, const Window& window=Window());

    bool integrate(const sensor_msgs::PointCloud2::ConstPtr& pcl_msg);
    sensor_msgs::PointCloud2::Ptr integratedCloud(std::string target_frame="", bool crop=true) const;
    
protected:
    int max_buffer_size_;
    std::string fixed_frame_;    
    tf2::BufferCore* tf_buffer_;
    Window window_;
    std::deque<sensor_msgs::PointCloud2::Ptr> cloud_buffer_;

    sensor_msgs::PointCloud2::Ptr crop_pcl(const sensor_msgs::PointCloud2::Ptr& pcl) const;
};

#endif // _PCL_INTEGRATOR_H_
