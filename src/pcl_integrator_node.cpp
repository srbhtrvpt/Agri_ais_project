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
#include <tf2_ros/buffer.h>
#include <tf2/convert.h>
#include <tf2/transform_datatypes.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include "window.h"
#define foreach BOOST_FOREACH

//#include <pcl/io/impl/synchronized_queue.hpp>

using namespace message_filters;
typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef sensor_msgs::PointCloud2Ptr SPCL;
//bool transform_pointcloud(PointCloud &cloud, std::string frame_id);
//bool is_inside(const PointT& p);

// ros parameters
int max_buffer_size;
bool crop_flag, bag_flag;
std::string fixed_frame, base_footprint, target_frame;

std::deque<PointCloud::Ptr, Eigen::aligned_allocator<PointT>> cloud_buffer;
std::deque<SPCL> cloud_buffer2;

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

sensor_msgs::PointCloud2Ptr crop_pcl2(sensor_msgs::PointCloud2Ptr cloud, tf2::BufferCore &tf_buffer)
{
    if (cloud->header.frame_id.compare(base_footprint) != 0)
    {
        //ROS_ERROR("transforming cloud for cropping");
        try
        {
            geometry_msgs::TransformStamped transform_stamped = tf_buffer.lookupTransform(base_footprint, cloud->header.frame_id, cloud->header.stamp);
            tf2::doTransform(*cloud, *cloud, transform_stamped);
        }
        catch (std::runtime_error &ex)
        {
            ROS_ERROR("transforming integrated cloud into base_footprint %s", base_footprint.c_str());
            return cloud;
        }
    }

    PointCloud::Ptr result(new PointCloud);
    for (sensor_msgs::PointCloud2ConstIterator<float> it(*cloud, "x"); it != it.end(); ++it)
    {
        PointT p;
        p.x = it[0];
        p.y = it[1];
        p.z = it[2];
        if (w.is_inside(p))
        {
            result->push_back(p);
        }
    }
    sensor_msgs::PointCloud2Ptr result_ros;
    pcl::toROSMsg(*result, *result_ros);
    result_ros->header = cloud->header;

    return result_ros;
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

void callback2(const sensor_msgs::PointCloud2ConstPtr &cloud_ros, tf2::BufferCore &tf_buffer) //change return type to sensormsgs pcl2
{
    //PointCloud::Ptr cloud(new PointCloud);
    // pcl::fromROSMsg(*cloud_ros, *cloud);
    sensor_msgs::PointCloud2Ptr cloud, integrated_cloud;
    geometry_msgs::TransformStamped transformStamped, transformStamped2;
    //tf2_ros::TransformListener *listener(tf_buffer);

    if (!cloud_buffer2.empty() && cloud_buffer2.back()->header.stamp == cloud_ros->header.stamp)
    {
        // ignore same point cloud (timestamp wise)
        return;
    }
    try
    {
        transformStamped = tf_buffer.lookupTransform(fixed_frame, cloud_ros->header.frame_id, cloud_ros->header.stamp);
        tf2::doTransform(*cloud_ros, *cloud, transformStamped);
    }
    catch (tf2::TransformException &ex)
    {
        ROS_ERROR("%s", ex.what());
        //ROS_ERROR("%s: cannot transform incoming pcl to fixed frame %s.", __func__, fixed_frame.c_str());
        return;
    }
    cloud_buffer2.push_back(cloud);
    int buffer_size = std::max(max_buffer_size, 1);

    while (cloud_buffer2.size() > buffer_size)
    {
        cloud_buffer2.pop_front();
    }
    if (cloud_buffer2.empty())
    {
        return;
    }

    //PointCloud::Ptr integrated_cloud(new PointCloud);

    for (size_t i = 0; i < cloud_buffer2.size(); ++i)
    {
        //*integrated_cloud += *(cloud_buffer2[i]);
        pcl::concatenatePointCloud(*integrated_cloud, *(cloud_buffer2[i]), *integrated_cloud);
    }
    integrated_cloud->header = cloud_buffer2.back()->header;
    if (crop_flag)
    {
        integrated_cloud = crop_pcl2(integrated_cloud, tf_buffer);
    }
    if (integrated_cloud->header.frame_id.compare(target_frame) != 0)
    {
        try
        {
            transformStamped2 = tf_buffer.lookupTransform(target_frame, integrated_cloud->header.frame_id, cloud_ros->header.stamp);
            tf2::doTransform(*integrated_cloud, *integrated_cloud, transformStamped2);
        }
        catch (std::runtime_error &ex)
        {
            ROS_ERROR("transforming integrated cloud into target_frame %s", target_frame.c_str());
            return;
        }
    }
    point_cloud_pub.publish(integrated_cloud);
    //return integrated_cloud;
}

int main(int argc, char *argv[])
{
    // This must be called before anything else ROS-related
    ros::init(argc, argv, "pcl_integrator_node");
    ros::NodeHandle nh;
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
        point_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("output_cloud", 1, true); //comment later

        rosbag::Bag input_bag, output_bag;
        std::vector<std::string> topics;
        tf2::BufferCore tf_buff(ros::Duration(70.0)); //change to bag size

        input_bag.open("/home/srbh/agrirobo_proj/with_pcls/largeplants.bag", rosbag::bagmode::Read);
        //output_bag.open("/home/srbh/agrirobo_proj/with_pcls/test.bag", rosbag::bagmode::Write);
        

        //give a tf buffer to listener. ( fill with the tf messages)
        //read all transforms one iteration before (prior for loop to add to tf buffer(size is duration of bag + some secs))
        topics.push_back("/tf");
        rosbag::View view(input_bag, rosbag::TopicQuery(topics));
        foreach (rosbag::MessageInstance const msg, view)
        {
            geometry_msgs::TransformStampedPtr t = msg.instantiate<geometry_msgs::TransformStamped>();
            if (t == NULL)
            {
                continue;
            }
            tf_buff.setTransform(*t, t->header.frame_id);
        }
        topics.pop_back();
        topics.push_back("/sensor/laser/vlp16/front/pointcloud_xyzi");

        rosbag::View view2(input_bag, rosbag::TopicQuery(topics));
        foreach (rosbag::MessageInstance const msg, view2)
        {
            sensor_msgs::PointCloud2ConstPtr pt_cloud = msg.instantiate<sensor_msgs::PointCloud2>();
            if (pt_cloud == NULL)
            {
                continue;
            }
            //std::cout << pt_cloud->header.stamp << std::endl;
            callback2(pt_cloud, tf_buff);
            //output_bag.write("/sensor/laser/vlp16/front/integrated_pointcloud_xyzi", ros::Time::now(), callback2(pt_cloud, tf_buff));
            ros::spinOnce();
        }
        
        
        //output_bag.close();
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
