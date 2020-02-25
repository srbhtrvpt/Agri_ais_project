#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <boost/bind.hpp>

//#include <pcl/io/impl/synchronized_queue.hpp>

using namespace message_filters;
ros::Publisher point_cloud_pub;
typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> PointCloud;
std::deque<PointCloud::Ptr, Eigen::aligned_allocator<PointT>> sourceClouds;
//std::vector < PointCloud, Eigen::aligned_allocator <PointT> > sourceClouds;
tf::TransformListener *listener;
tf::TransformBroadcaster *br;
int decay_size;
sensor_msgs::PointCloud2::Ptr output_cloud(new sensor_msgs::PointCloud2);


void callback(const sensor_msgs::PointCloud2ConstPtr &cloud) //const nav_msgs::Odometry::ConstPtr& odom1
{
    tf::StampedTransform transform;
    PointCloud::Ptr cloud_in(new PointCloud);
    PointCloud::Ptr cloud_out(new PointCloud);
    PointCloud::Ptr cloud_result(new PointCloud);


    //pcl::SynchronizedQueue < PointCloud> sourceClouds;
    //std::vector < PointCloud::Ptr, Eigen::aligned_allocator <PointCloud ::Ptr > > sourceClouds;
    //std::vector<pcl::PointCloud<pcl::PointXYZ>, Eigen::aligned_allocator<pcl::PointXYZ> > sourceClouds;

    pcl::fromROSMsg(*cloud, *cloud_in);
    //point_cloud_pub.publish(*cloud_in);
    sourceClouds.push_back(cloud_in);
    //std::cout << sourceClouds.size() << "\n";
    //ROS_INFO_STREAM(ros::Time::now());

    if (sourceClouds.size() >= decay_size)
    {

        int i = sourceClouds.size() - 1;
        *cloud_result = *sourceClouds[i];

        cloud_result->header = sourceClouds[i]->header;
        for (int j = 0; j < i; j++)
        {
            try
            {
                //std::cout << "now" << sourceClouds[i]->header.stamp << "------" << i << "\n";
                //std::cout << sourceClouds[j]->header.stamp << "------" << j << "\n";

                //int x = ;
                ros::Time now(sourceClouds[i]->header.stamp / 1e6, fmod(sourceClouds[i]->header.stamp, 1e6));
                ros::Time past(sourceClouds[j]->header.stamp / 1e6, fmod(sourceClouds[j]->header.stamp, 1e6));
                /*listener->waitForTransform(sourceClouds[i].header.frame_id, now,
                              sourceClouds[j].header.frame_id, past,
                               odom1->header.frame_id, ros::Duration(0.01)); */
                listener->lookupTransform(sourceClouds[i]->header.frame_id, now,
                                          sourceClouds[j]->header.frame_id, past,
                                          "odom", transform);
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
    pcl::toROSMsg(*cloud_result, *output_cloud);
    point_cloud_pub.publish(output_cloud);
}

int main(int argc, char *argv[])
{
    // This must be called before anything else ROS-related
    ros::init(argc, argv, "transform1_node");

    /*tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    geometry_msgs::TransformStamped transformStamped;
    static tf2_ros::TransformBroadcaster br2; */
    ros::NodeHandle nh;
    listener = new tf::TransformListener();
    br = new tf::TransformBroadcaster();
    /*message_filters::Subscriber<nav_msgs::Odometry> odom_sub(nh, "/robot/odom", 1000);
    message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub(nh, "/sensor/laser/vlp16/front/pointcloud_xyzi", 1000);
    typedef sync_policies::ApproximateTime<nav_msgs::Odometry, sensor_msgs::PointCloud2> MySyncPolicy;
      //typedef sync_policies::ExactTime<nav_msgs::Odometry, sensor_msgs::PointCloud2> MySyncPolicy;
    
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), odom_sub, cloud_sub);
    sync.registerCallback(boost::bind(&callback, _1, _2));*/

    nh.getParam("/transform1_node/decay_size", decay_size);

    // Create a ROS node handle

    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/sensor/laser/vlp16/front/pointcloud_xyzi", 100, callback);

    point_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("point_cloud_decay", 1, true);
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