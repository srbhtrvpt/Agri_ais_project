#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/PointCloud2.h>
// #include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/crop_box.h>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <boost/bind.hpp>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>
#include <pcl_ros/point_cloud.h>
#include "window.h"
#define foreach BOOST_FOREACH

//#include <pcl/io/impl/synchronized_queue.hpp>

using namespace message_filters;
typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> PointCloud;

//bool transform_pointcloud(PointCloud &cloud, std::string frame_id);
//bool is_inside(const PointT& p);

// ros parameters
int max_buffer_size;
bool crop_flag, bag_flag;
std::string fixed_frame, base_footprint, target_frame;

//std::deque<PointCloud::Ptr, Eigen::aligned_allocator<PointT> > sourceClouds;
std::deque<PointCloud::Ptr, Eigen::aligned_allocator<PointT>> cloud_buffer;
tf::TransformListener *listener;
//tf::TransformBroadcaster *br;

ros::Publisher point_cloud_pub;
Window<PointT> w;

/*bool is_inside(const PointT& p)
{
    float min_x = offset_x;
    float max_x = offset_x + size_x;
    float min_y = offset_y - 0.5*size_y;
    float max_y = offset_y + 0.5*size_y;
    
    return (p.x >= min_x && p.x <= max_x) && (p.y >= min_y && p.y <= max_y);
} */

PointCloud::Ptr crop_pcl(PointCloud::Ptr cloud)
{
    if (cloud->header.frame_id.compare(base_footprint) != 0)
    {
        //ROS_ERROR("transforming cloud for cropping");
        if (!pcl_ros::transformPointCloud(base_footprint, *cloud, *cloud, *listener))
        {
            return cloud;
        }
    }

    PointCloud::Ptr result(new PointCloud);
    result->header = cloud->header;
    for (size_t i = 0; i < cloud->size(); ++i)
    {
        const PointT &p = (*cloud)[i];
        if (w.is_inside(p))
        {
            result->push_back(p);
        }
    }
    return result;
}

void callback(const sensor_msgs::PointCloud2ConstPtr &cloud_ros)
{
    PointCloud::Ptr cloud(new PointCloud);
    pcl::fromROSMsg(*cloud_ros, *cloud);

    if (!cloud_buffer.empty() && cloud_buffer.back()->header.stamp == cloud->header.stamp)
    {
        // ignore same point cloud (timestamp wise)
        return;
    }

    if (!pcl_ros::transformPointCloud(fixed_frame, *cloud, *cloud, *listener))
    {
        ROS_ERROR("%s: cannot transform incoming pcl to fixed frame %s.", __func__, fixed_frame.c_str());
        return;
    }
    cloud_buffer.push_back(cloud);

    int buffer_size = std::max(max_buffer_size, 1);

    while (cloud_buffer.size() > buffer_size)
    {
        cloud_buffer.pop_front();
    }
    if (cloud_buffer.empty())
    {
        return;
    }

    PointCloud::Ptr integrated_cloud(new PointCloud);
    for (size_t i = 0; i < cloud_buffer.size(); ++i)
    {
        *integrated_cloud += *(cloud_buffer[i]);
    }
    integrated_cloud->header = cloud_buffer.back()->header;
    if (crop_flag)
    {
        integrated_cloud = crop_pcl(integrated_cloud);
    }
    if (integrated_cloud->header.frame_id.compare(target_frame) != 0)
    {
        if (!pcl_ros::transformPointCloud(target_frame, *integrated_cloud, *integrated_cloud, *listener))
        {
            ROS_ERROR("transforming integrated cloud into target_frame %s", target_frame.c_str());
            return;
        }
    }
    point_cloud_pub.publish(integrated_cloud);
}

int main(int argc, char *argv[])
{
    // This must be called before anything else ROS-related
    ros::init(argc, argv, "pcl_integrator_node");
    ros::NodeHandle nh;
    listener = new tf::TransformListener();
    float size_x, size_y, offset_x, offset_y;

    ros::NodeHandle nhPriv("~");

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
    nhPriv.param("bag_flag", bag_flag, false);

    nhPriv.param<std::string>("fixed_frame", fixed_frame, "odom");
    nhPriv.param<std::string>("base_footprint", base_footprint, "base_footprint");
    target_frame = base_footprint;
    w = Window<PointT>(size_x, size_y, offset_x, offset_y);

    if (bag_flag)
    {

        rosbag::Bag input_bag, output_bag;
        std::vector<std::string> topics;
        input_bag.open("/home/srbh/agrirobo_proj/with_pcls/largeplants.bag", rosbag::bagmode::Read);
        //output_bag.open("/home/srbh/agrirobo_proj/with_pcls/test.bag", rosbag::bagmode::Write);
        topics.push_back("/sensor/laser/vlp16/front/pointcloud_xyzi");
        topics.push_back("/tf");

        //give a tf buffer to listener. ( fill with the tf messages)
        //read all transforms one iteration before (prior for loop to add to tf buffer(size is duration of bag + some secs))
        rosbag::View view(input_bag, rosbag::TopicQuery(topics));
        //listener = *listener(ros::Duration(70));

        foreach (rosbag::MessageInstance const msg, view)
        {
            sensor_msgs::PointCloud2ConstPtr pt_cloud = msg.instantiate<sensor_msgs::PointCloud2>();
            if (pt_cloud == NULL)
            {
                continue;
            }
            std::cout << pt_cloud->header.stamp << std::endl;
            //callback(pt_cloud);
            ros::spinOnce();
        }
        std::cout << "done" << std::endl;
    }

    else
    {
        ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("input_cloud", 10, callback); // 100 in buffer is a bit much?
        point_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("output_cloud", 1, true);         // you can use remap in the launch file to set the correct runtime topics!
        ros::spin();
    }

    return 0;
}
