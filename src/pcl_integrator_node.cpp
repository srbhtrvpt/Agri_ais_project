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

typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> PointCloud;

// ros parameters
int decay_size;
int max_buffer_size = 20;
//float area_box, offset_x, offset_y, x_box, y_box;

float size_x = 8.f;
float size_y = 6.f;
float offset_x = 1.f;
float offset_y = 0.f;

bool crop_flag = true;
std::string fixed_frame = "odom";
std::string base_footprint = "base_footprint";
std::string target_frame = base_footprint;

std::deque<PointCloud::Ptr, Eigen::aligned_allocator<PointT> > sourceClouds;

std::deque<PointCloud::Ptr, Eigen::aligned_allocator<PointT> > cloud_buffer;

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
        if(!pcl_ros::transformPointCloud(base_footprint, *cloud, *cloud, *listener)){
            return cloud;
        }
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

void callback2(const sensor_msgs::PointCloud2ConstPtr &cloud_ros)
{
    PointCloud::Ptr cloud(new PointCloud);
    pcl::fromROSMsg(*cloud_ros, *cloud);

    if(!cloud_buffer.empty() && cloud_buffer.back()->header.stamp == cloud->header.stamp){
        // ignore same point cloud (timestamp wise)
        return;
    }
    
    if(!pcl_ros::transformPointCloud(fixed_frame, *cloud, *cloud, *listener)){
        ROS_ERROR("%s: cannot transform incoming pcl to fixed frame %s.", __func__, fixed_frame.c_str());
        return;
    }
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
    }
    integrated_cloud->header = cloud_buffer.back()->header;
    if(crop_flag){
        integrated_cloud = crop_pcl(integrated_cloud);
    }
    if(integrated_cloud->header.frame_id.compare(target_frame) != 0){
        if(!pcl_ros::transformPointCloud(target_frame, *integrated_cloud, *integrated_cloud, *listener)){
            ROS_ERROR("transforming integrated cloud into target_frame %s", target_frame.c_str());
            return;
        }
    }
    point_cloud_pub.publish(integrated_cloud);
}

void callback(const sensor_msgs::PointCloud2ConstPtr &cloud) //const nav_msgs::Odometry::ConstPtr& odom1
{
    tf::StampedTransform transform;
    PointCloud::Ptr cloud_in(new PointCloud);
    

    PointCloud::Ptr cloud_out(new PointCloud);
    PointCloud::Ptr cloud_result(new PointCloud);

    pcl::fromROSMsg(*cloud, *cloud_in);

    PointCloud::Ptr cloud_inter = crop_pcl(cloud_in);
    cloud_inter->header = cloud_in->header;
    //pcl::SynchronizedQueue < PointCloud> sourceClouds;
    //std::vector < PointCloud::Ptr, Eigen::aligned_allocator <PointCloud ::Ptr > > sourceClouds;
    //std::vector<pcl::PointCloud<pcl::PointXYZ>, Eigen::aligned_allocator<pcl::PointXYZ> > sourceClouds;

    //point_cloud_pub.publish(*cloud_in);
    sourceClouds.push_back(cloud_inter);
    //std::cout << sourceClouds.size() << "\n";
    //ROS_INFO_STREAM(ros::Time::now());

    if (sourceClouds.size() >= decay_size)
    {

        int i = sourceClouds.size() - 1; // why not pop the front here and then iterate over the whole queue?
        *cloud_result = *sourceClouds[i]; // here you already copy the data into cloud_result...
        cloud_result->header = sourceClouds[i]->header; // so this is unnecessary
        
        for (int j = 0; j < i; j++)
        {
            try
            {
                //std::cout << "now" << sourceClouds[i]->header.stamp << "------" << i << "\n";
                //std::cout << sourceClouds[j]->header.stamp << "------" << j << "\n";

                //int x = ;
                ros::Time now(sourceClouds[i]->header.stamp / 1e6, fmod(sourceClouds[i]->header.stamp, 1e6)); // this seems unnecessary...
                ros::Time past(sourceClouds[j]->header.stamp / 1e6, fmod(sourceClouds[j]->header.stamp, 1e6)); // this seems unnecessary...
                /*listener->waitForTransform(sourceClouds[i].header.frame_id, now,
                              sourceClouds[j].header.frame_id, past,
                               odom1->header.frame_id, ros::Duration(0.01)); */
                listener->lookupTransform(sourceClouds[i]->header.frame_id, now,
                                          sourceClouds[j]->header.frame_id, past,
                                          "odom", transform); // this can block for some time and make your code very slow!!!
                // also if the point cloud is too old, there is no transform in the buffer anymore... and probably that's why the pcl is so sparse...
            }
            catch (std::runtime_error &ex)
            {
                ROS_ERROR("%s", ex.what());
            }
            pcl_ros::transformPointCloud(*sourceClouds[j], *cloud_out, transform);
            *cloud_result += *cloud_out;
        }

        sourceClouds.pop_front();
        /*assert(!sourceClouds.empty());
    sourceClouds.erase(sourceClouds.begin());*/
    }
    /*if(!cloud_result.empty())
    {    
    }*/
    //std::cout << "header" << cloud_result->header.frame_id ;
    
    sensor_msgs::PointCloud2 output_cloud;
    pcl::toROSMsg(*cloud_result, output_cloud);
    point_cloud_pub.publish(output_cloud);
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
    //nhPriv.getParam("decay_size", decay_size);
    //nhPriv.getParam("area_box", area_box);
    //nhPriv.getParam("offset_x", offset_x);
    //nhPriv.getParam("offset_y", offset_y);

    if(!nhPriv.getParam("max_buffer_size", max_buffer_size)){
        ROS_ERROR("max_buffer_size was not set!");
        return 1;
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
