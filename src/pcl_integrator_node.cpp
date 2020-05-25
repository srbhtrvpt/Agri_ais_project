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
float area_box, offset_x, offset_y, x_box, y_box;

std::deque<PointCloud::Ptr, Eigen::aligned_allocator<PointT> > sourceClouds;
tf::TransformListener *listener;
//tf::TransformBroadcaster *br;

ros::Publisher point_cloud_pub;

PointCloud::Ptr crop_pcl(PointCloud::Ptr cloud_in)    
{
    // ensure that cloud is in correct frame???
    // ie base_footprint???
    
    PointCloud::Ptr cloud_inter(new PointCloud);
    // why use this filter? just write your own "crop"
    // you are introducing constraints on z, that we do not want!
    // (setting z range implicitly to [-2,2])
    // not a nice behavior of your code!!!
    pcl::CropBox<PointT> cropBoxFilter(true);
    cropBoxFilter.setInputCloud(cloud_in);

    Eigen::Vector4f min_pt(-x_box, -y_box, -2.0, 1.0f);
    Eigen::Vector4f max_pt(x_box, y_box, 2.0, 1.0f);

    // Cropbox slighlty bigger then bounding box of points
    cropBoxFilter.setMin(min_pt);
    cropBoxFilter.setMax(max_pt);

    // Indices
    std::vector<int> indices;
    cropBoxFilter.filter(indices);

    // Cloud
   
    cropBoxFilter.filter(*cloud_inter);
    return cloud_inter;

}

void callback(const sensor_msgs::PointCloud2ConstPtr &cloud) //const nav_msgs::Odometry::ConstPtr& odom1
{

    // Better approach:
    // 1) transform incoming pcls into odom frame
    // 2) store them in your deque
    // 3) pop deque until size <= decay_size
    // 4) integrate pcls
    // 5) transform result pcl into target frame (= base_footprint?)
    // 6) crop using detection window parameters (make it optional using a ros parameter)
    // 7) publish the resulting pcl

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

    nh.getParam("/pcl_integrator_node/decay_size", decay_size);
    nh.getParam("/pcl_integrator_node/area_box", area_box);
    nh.getParam("/pcl_integrator_node/offset_x", offset_x);
    nh.getParam("/pcl_integrator_node/offset_y", offset_y);

    x_box = sqrt((4*area_box)/3) + offset_x; // why so complicated?
    y_box = sqrt((3*area_box)/4) + offset_y; // just implement your own crop function!!!


    // Create a ROS node handle
    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("input_cloud", 10, callback); // 100 in buffer is a bit much?
    point_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("output_cloud", 1, true); // you can use remap in the launch file to set the correct runtime topics!
    ros::spin();
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
