#include "pcl_integrator.h"

#include <geometry_msgs/TransformStamped.h>
#include <pcl_conversions/pcl_conversions.h> // pcl::concatenate
#include <tf2_sensor_msgs/tf2_sensor_msgs.h> // tf2::doTransform for PointCloud2

PclIntegrator::PclIntegrator(std::string fixed_frame, tf2::BufferCore* tf_buffer, int max_buffer_size, const Window& window) :
    max_buffer_size_(max_buffer_size)
    , fixed_frame_(fixed_frame)
    , tf_buffer_(tf_buffer)
    , window_(window)
{
}

bool PclIntegrator::integrate(const sensor_msgs::PointCloud2::Ptr& pcl_msg)
{
    geometry_msgs::TransformStamped transformStamped;
    if (!cloud_buffer_.empty() && cloud_buffer_.back()->header.stamp == pcl_msg->header.stamp)
    {
        // ignore same point cloud (timestamp wise)
        return false;
    }
    try
    {
        transformStamped = tf_buffer_->lookupTransform(fixed_frame_, pcl_msg->header.frame_id, pcl_msg->header.stamp);
    }
    catch (tf2::TransformException &ex)
    {
        ROS_ERROR("%s", ex.what());
        ROS_ERROR("%s: cannot transform incoming pcl to fixed frame %s.", __func__, fixed_frame_.c_str());
        return false;
    }

    tf2::doTransform(*pcl_msg, *pcl_msg, transformStamped);
    cloud_buffer_.push_back(pcl_msg);

    int buffer_size = std::max(max_buffer_size_, 1);
    while (cloud_buffer_.size() > buffer_size)
    {
        cloud_buffer_.pop_front();
    }
    if (cloud_buffer_.empty())
    {
        return false;
    }
    return true;
}

sensor_msgs::PointCloud2::Ptr PclIntegrator::integratedCloud(std::string target_frame, bool crop) const
{    
    sensor_msgs::PointCloud2::Ptr integrated_cloud(new sensor_msgs::PointCloud2());

    for (size_t i = 0; i < cloud_buffer_.size(); ++i)
    {
        pcl::concatenatePointCloud(*integrated_cloud, *(cloud_buffer_[i]), *integrated_cloud);
    }
    integrated_cloud->header = cloud_buffer_.back()->header;

    if (crop)
    {
        integrated_cloud = crop_pcl(integrated_cloud);
    }

    if (!target_frame.empty() && integrated_cloud->header.frame_id.compare(target_frame) != 0)
    {
        geometry_msgs::TransformStamped transformStamped;
        try
        {
            transformStamped = tf_buffer_->lookupTransform(target_frame, integrated_cloud->header.frame_id, integrated_cloud->header.stamp);
        }
        catch (std::runtime_error &ex)
        {
            ROS_ERROR("%s", ex.what());
            ROS_ERROR("transforming integrated cloud into target_frame %s", target_frame.c_str());
            return integrated_cloud;
        }
        tf2::doTransform(*integrated_cloud, *integrated_cloud, transformStamped);
    }
    return integrated_cloud;
}

sensor_msgs::PointCloud2::Ptr PclIntegrator::crop_pcl(const sensor_msgs::PointCloud2::Ptr& cloud) const
{
    sensor_msgs::PointCloud2::Ptr cropped_pcl(new sensor_msgs::PointCloud2());
    //for (sensor_msgs::PointCloud2ConstIterator<float> it(*cloud, "x"); it != it.end(); ++it)
    //{
    //    // this seems strange...
    //    PointT p;
    //    p.x = it[0];
    //    p.y = it[1];
    //    p.z = it[2];
    //    if (w.is_inside(p))
    //    {
    //        result->push_back(p);
    //    }
    //}
    return cropped_pcl;
}
