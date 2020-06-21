#include "pcl_integrator.h"

#include <geometry_msgs/TransformStamped.h>
#include <pcl_conversions/pcl_conversions.h> // pcl::concatenate
#include <tf2_sensor_msgs/tf2_sensor_msgs.h> // tf2::doTransform for PointCloud2

PclIntegrator::PclIntegrator(std::string fixed_frame, tf2_ros::Buffer* tf_buffer, int max_buffer_size) :
    PclIntegrator(fixed_frame, Window(fixed_frame), tf_buffer, max_buffer_size)
{
}

PclIntegrator::PclIntegrator(std::string fixed_frame, const Window& window, tf2_ros::Buffer* tf_buffer, int max_buffer_size) :
    max_buffer_size_(max_buffer_size)
    , fixed_frame_(fixed_frame)
    , tf_buffer_(tf_buffer)
    , window_(window)
{
}

/*
bool PclIntegrator::transformPCL(sensor_msgs::PointCloud2::Ptr pcl_msg, std::string frame)
{
    geometry_msgs::TransformStamped transformStamped;
    try
    {
        transformStamped = tf_buffer_->lookupTransform(frame, pcl_msg->header.frame_id, pcl_msg->header.stamp);
    }
    catch (tf2::TransformException &ex)
    {
        ROS_ERROR("%s", ex.what());
        ROS_ERROR("%s: cannot transform incoming pcl to frame %s.", __func__, frame.c_str());
        return false;
    }
    tf2::doTransform(*pcl_msg, *pcl_msg, transformStamped);
    return true;
}
*/
bool PclIntegrator::integrate(const sensor_msgs::PointCloud2::ConstPtr& pcl_msg_orig)
{
    sensor_msgs::PointCloud2::Ptr pcl_msg(new sensor_msgs::PointCloud2);
    *pcl_msg = *pcl_msg_orig;
    if (!cloud_buffer_.empty() && cloud_buffer_.back()->header.stamp == pcl_msg->header.stamp)
    {
        // ignore same point cloud (timestamp wise)
        return false;
    }
    geometry_msgs::TransformStamped transformStamped;
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
        ROS_ERROR("%s: cloud buffer empty!", __func__);
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
        if(integrated_cloud->header.frame_id.compare(window_.frame_id()) != 0){
            geometry_msgs::TransformStamped transformStamped;
            try
            {
                transformStamped = tf_buffer_->lookupTransform(window_.frame_id(), integrated_cloud->header.frame_id, integrated_cloud->header.stamp);
            }
            catch (std::runtime_error &ex)
            {
                ROS_ERROR("%s", ex.what());
                ROS_ERROR("transforming integrated cloud into window frame %s", window_.frame_id().c_str());
                return integrated_cloud;
            }
            tf2::doTransform(*integrated_cloud, *integrated_cloud, transformStamped);
        }
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

bool hasField(const sensor_msgs::PointCloud2::Ptr& cloud, std::string fieldname)
{
    const std::vector<sensor_msgs::PointField>& fields = cloud->fields;
    for(size_t i = 0; i < fields.size(); ++i){
        if(fields[i].name.compare(fieldname) == 0){
            return true;
        }
    }
    ROS_ERROR("%s: cloud has no field %s!", __func__, fieldname.c_str());
    return false;
}

sensor_msgs::PointCloud2::Ptr PclIntegrator::crop_pcl(const sensor_msgs::PointCloud2::Ptr& cloud) const
{
    // check that cloud contains x and y fields
    if(!hasField(cloud, "x") || !hasField(cloud, "y")){
        return cloud;
    }
    
    sensor_msgs::PointCloud2::Ptr cropped_pcl(new sensor_msgs::PointCloud2());
    cropped_pcl->header = cloud->header;
    cropped_pcl->fields = cloud->fields;
    cropped_pcl->is_bigendian = cloud->is_bigendian;
    int point_step = cropped_pcl->point_step = cloud->point_step;
    cropped_pcl->is_dense = cloud->is_dense;
    cropped_pcl->height = 1;

    const std::vector<uint8_t>& cloud_data = cloud->data;
    std::vector<uint8_t>& cropped_data = cropped_pcl->data;
    
    sensor_msgs::PointCloud2ConstIterator<float> it_x(*cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> it_y(*cloud, "y");

    int cloud_index = 0;
    int cropped_pcl_index = 0;
    for (; it_x != it_x.end(); ++it_x)
    {
        if (window_.is_inside(*it_x, *it_y))
        {
            // you need to make sure, that you push all point data into the new point cloud!!
            int data_start = cloud_index*point_step;
            int data_end = (cloud_index+1)*point_step;
            cropped_data.insert(cropped_data.end(), cloud_data.begin()+data_start, cloud_data.begin()+data_end);
            cropped_pcl_index++;
        }
        ++it_y;
        cloud_index++;
    }
    cropped_pcl->row_step = cropped_pcl_index;
    cropped_pcl->width = cropped_pcl_index;
    return cropped_pcl;
}
