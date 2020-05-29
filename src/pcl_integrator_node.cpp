#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
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

//#include <pcl/io/impl/synchronized_queue.hpp>

using namespace message_filters;

ros::Time to_ros_time(pcl::uint64_t stamp);
bool transform_pointcloud(PointCloud& cloud, std::string frame_id);
bool is_inside(const PointT& p);

typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> PointCloud;

// ros parameters
int max_buffer_size;
//float area_box, offset_x, offset_y, x_box, y_box;
float size_x, size_y, offset_x, offset_y;
bool crop_flag = true;
std::string fixed_frame = "odom";
std::string base_footprint = "base_footprint";
std::string target_frame = base_footprint;

//std::deque<PointCloud::Ptr, Eigen::aligned_allocator<PointT> > sourceClouds;
std::deque<PointCloud::Ptr, Eigen::aligned_allocator<PointT> > cloud_buffer;

ros::Time current_stamp;
tf::TransformListener *listener;
//tf::TransformBroadcaster *br;

ros::Publisher point_cloud_pub;



//const Window window(offset_x, ...);
// window.is_inside(p);

bool is_inside(const PointT& p)
{
    float min_x = offset_x;
    float max_x = offset_x + size_x;
    float min_y = offset_y - 0.5*size_y;
    float max_y = offset_y + 0.5*size_y;
    
    return (p.x >= min_x && p.x <= max_x) && (p.y >= min_y && p.y <= max_y);
}
        

PointCloud::Ptr crop_pcl(PointCloud::Ptr cloud)    
{
    if(cloud->header.frame_id.compare(base_footprint) != 0){
        //ROS_ERROR("transforming cloud for cropping");
        transform_pointcloud(*cloud, base_footprint);
    }

    PointCloud::Ptr result(new PointCloud);
    result->header = cloud->header;
    for(size_t i = 0; i < cloud->size(); ++i){
        const PointT& p = (*cloud)[i];
        if(is_inside(p)){
            result->push_back(p);
        }
    }
    return result;
}

/// DO NOT USE THIS, not enough accuracy!!!
/// just remember original time stamp from pointcloud2 msg as "current_stamp"
ros::Time to_ros_time(pcl::uint64_t stamp)
{
    ros::Time t(stamp * 1e-6, fmod(stamp, 1e6)*1e3); // need to convert musec to nsec
    //ROS_ERROR("converted ROS time: %.9f", t.toSec());
    
    return ros::Time(stamp * 1e-6, fmod(stamp, 1e6));
}

bool transform_pointcloud(PointCloud& cloud, std::string frame_id)
{
    tf::StampedTransform transform;
    bool success = true;
    try
    {
        //listener->lookupTransform(frame_id, cloud.header.frame_id, to_ros_time(cloud.header.stamp), transform);
        listener->lookupTransform(frame_id, cloud.header.frame_id, current_stamp, transform);
    }
    catch (std::runtime_error &ex)
    {
        ROS_ERROR("%s", ex.what());
        success = false;
    }
    if(!success){
        return false;
    }
    PointCloud copy_cloud = cloud;
    pcl_ros::transformPointCloud(copy_cloud, cloud, transform);// check if header is correct after transforming
    cloud.header.frame_id = frame_id;        
    //ROS_ERROR("input frame id: %s, output frame_id: %s", copy_cloud.header.frame_id.c_str(), cloud.header.frame_id.c_str());
    return true;
}

void callback2(const sensor_msgs::PointCloud2ConstPtr &cloud_ros)
{
    //ROS_ERROR("ROS timestamp: %.9f", cloud_ros->header.stamp.toSec());
    current_stamp = cloud_ros->header.stamp;
    
    if(!cloud_buffer.empty() && to_ros_time(cloud_buffer.back()->header.stamp).toSec() == fmod(cloud_ros->header.stamp.toSec(),1e3)*1e3){
        // ignore same point cloud (timestamp wise)
        return;
    }
    
    PointCloud::Ptr cloud(new PointCloud);
    pcl::fromROSMsg(*cloud_ros, *cloud);

    //ROS_ERROR("transforming cloud");
    if(!transform_pointcloud(*cloud, fixed_frame)){
        ROS_ERROR("%s: cannot transform incoming pcl to fixed frame %s.", __func__, fixed_frame.c_str());
        return;
    }
    //ROS_ERROR("pushing cloud with frame_id %s", cloud->header.frame_id.c_str());
    cloud_buffer.push_back(cloud);

    int buffer_size = std::max(max_buffer_size, 1);
    
    while(cloud_buffer.size() > buffer_size){
        cloud_buffer.pop_front();
    }
    if(cloud_buffer.empty()){
        return;
    }
    
    PointCloud::Ptr integrated_cloud(new PointCloud);
    for(size_t i = 0; i < cloud_buffer.size(); ++i){
        *integrated_cloud += *(cloud_buffer[i]);
        //const PointCloud& cld = *(cloud_buffer[i]);
        //integrated_cloud->insert(integrated_cloud->end(), cld.begin(), cld.end());
    }
    integrated_cloud->header = cloud_buffer.back()->header;
    if(crop_flag){
        integrated_cloud = crop_pcl(integrated_cloud);
    }
    if(integrated_cloud->header.frame_id.compare(target_frame) != 0){
        //ROS_ERROR("transforming integrated_cloud");
        transform_pointcloud(*integrated_cloud, target_frame);
    }

    sensor_msgs::PointCloud2 integrated_cloud_ros;
    pcl::toROSMsg(*integrated_cloud, integrated_cloud_ros);
    integrated_cloud_ros.header.stamp = current_stamp;
    point_cloud_pub.publish(integrated_cloud_ros);
}


int main(int argc, char *argv[])
{
    // This must be called before anything else ROS-related
    ros::init(argc, argv, "pcl_integrator_node");

    /*tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    geometry_msgs::TransformStamped transformStamped;
    static tf2_ros::TransformBroadcaster br2; */
    ros::NodeHandle nh;
    listener = new tf::TransformListener();
    // br = new tf::TransformBroadcaster(); // you do not need that...
    
    /*message_filters::Subscriber<nav_msgs::Odometry> odom_sub(nh, "/robot/odom", 1000);
    message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub(nh, "/sensor/laser/vlp16/front/pointcloud_xyzi", 1000);
    typedef sync_policies::ApproximateTime<nav_msgs::Odometry, sensor_msgs::PointCloud2> MySyncPolicy;
      //typedef sync_policies::ExactTime<nav_msgs::Odometry, sensor_msgs::PointCloud2> MySyncPolicy;
    
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), odom_sub, cloud_sub);
    sync.registerCallback(boost::bind(&callback, _1, _2));*/

    ros::NodeHandle nhPriv("~");
    

    if(!nhPriv.getParam("max_buffer_size", max_buffer_size)){
        ROS_ERROR("max_buffer_size was not set!");
        return 1;
    }
    if(!nhPriv.getParam("size_x", size_x)){
        size_x = 8.f;
    }
    if(!nhPriv.getParam("size_y", size_y)){
        size_y = 6.f;
    }
    if(!nhPriv.getParam("offset_x", offset_x)){
        offset_x = 1.f;
    }
    if(!nhPriv.getParam("offset_y", offset_y)){
        offset_y = 0.f;
    }
    if(!nhPriv.getParam("fixed_frame", fixed_frame)){
        ROS_ERROR("fixed_frame was not set, setting to default");
        fixed_frame = "odom";
    }
    if(!nhPriv.getParam("base_footprint", base_footprint)){
        ROS_ERROR("base_footprint was not set, setting to default");
        base_footprint = "base_footprint";
    }

    //x_box = sqrt((4*area_box)/3) + offset_x; // why so complicated?
    //y_box = sqrt((3*area_box)/4) + offset_y; // just implement your own crop function!!!


    // Create a ROS node handle
    //ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("input_cloud", 10, callback); // 100 in buffer is a bit much?
    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("input_cloud", 10, callback2); // 100 in buffer is a bit much?
    point_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("output_cloud", 1, true); // you can use remap in the launch file to set the correct runtime topics!
    ros::spin();
    return 0;
}

//listener.waitForTransform('/sensor/laser/vlp16/front/pointcloud_xyzi',,'/sensor/laser/vlp16/front/pointcloud_xyzi',, , ros::Time(0), ros::Duration(10.0) );
/*ros::Time past = ros::Time::now() - ros::Duration(5.0); 
        transformStamped = tfBuffer.lookupTransform("/sensor/laser/vlp16/front/pointcloud_xyzi", ros::Time::now(), "/sensor/laser/vlp16/front/pointcloud_xyzi", past, "/robot/odom", ros::Duration(1.0));*/
/*ros::Time now = ros::Time::now();
        ros::Time past = now - ros::Duration(5.0);
    listener.waitForTransform("/sensor/laser/vlp16/front/pointcloud_xyzi", now,
                              "/sensor/laser/vlp16/front/pointcloud_xyzi", past,
                              "/robot/odom", ros::Duration(1.0));
    listener.lookupTransform("/sensor/laser/vlp16/front/pointcloud_xyzi", now,
                             "/sensor/laser/vlp16/front/pointcloud_xyzi", past,
                             "/robot/odom", transform); */