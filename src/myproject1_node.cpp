#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <boost/bind.hpp>
#include <image_geometry/pinhole_camera_model.h>
#include <opencv/cv.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <ros/console.h>

using namespace message_filters;
ros::Publisher point_cloud_pub;
//ros::Publisher colored_pt_pub;
image_geometry::PinholeCameraModel cam_model_;
ros::Subscriber sub;
bool cam_model_flag = false;

typedef pcl::PointXYZ PointT;
typedef pcl::PointXYZRGB PointC;
typedef pcl::PointCloud<PointC> ColoredPointCloud;
typedef pcl::PointCloud<PointT> PointCloud;

void callback(const sensor_msgs::ImageConstPtr &image, const sensor_msgs::PointCloud2ConstPtr &cloud)
{

    PointCloud::Ptr cloud_out(new PointCloud);
    //PointCloud cloud_out;
    PointCloud::Ptr cloud_in(new PointCloud);
    ColoredPointCloud::Ptr cloud_colored(new ColoredPointCloud);
    PointCloud::iterator it;
    PointC color_pt;
    tf::TransformListener listener;
    static tf::TransformBroadcaster br;
    cv::Point2d uv;
    cv::Mat image_in;
    cv_bridge::CvImagePtr input_bridge;
    tf::StampedTransform transform;

    try
    {
        input_bridge = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
        image_in = input_bridge->image;
    }
    catch (cv_bridge::Exception &ex)
    {
        ROS_ERROR("[draw_frames] Failed to convert image");
    }

    pcl::fromROSMsg(*cloud, *cloud_in);

    try
    {
        listener.waitForTransform(image->header.frame_id, cloud->header.frame_id, ros::Time(0), ros::Duration(10.0));
        listener.lookupTransform(image->header.frame_id, cloud->header.frame_id, ros::Time(0), transform);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s", ex.what());
    }

    //br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/base_sensor_blackfly_optical_link", "/base_sensor_fx8_laser_link"));

    //todo: convert sensr msgs to pointcloud pcl and transform to non stamped(access it)

    pcl_ros::transformPointCloud(*cloud_in, *cloud_out, transform);
    cloud_out->header.frame_id = image->header.frame_id;
    //pcl_conversions::toPCL(cloud_out->header, image->header);
    if (cam_model_flag == true)
    {

        // for(pcl::PointCloud<pcl::PointXYZRGB>::iterator it = cloud->begin(); it!= cloud->end(); it++)
        for (it = cloud_out->points.begin(); it < cloud_out->points.end(); it++)
        {
            //tf::Point pt = transform.getOrigin();
            //cv::Point3d pt_cv(pt.x(), pt.y(), pt.z());
            cv::Point3d pt_cv(it->x, it->y, it->z);
            uv = cam_model_.project3dToPixel(pt_cv);
            //printf("%f",uv.x);
            if (uv.x > 0 && uv.x < image_in.cols && uv.y < image_in.rows && uv.y > 0)
            {
                cv::Point3_<uchar> *pix = image_in.ptr<cv::Point3_<uchar>>(uv.y, uv.x);
                color_pt.x = it->x;
                color_pt.y = it->y;
                color_pt.z = it->z;
                uint32_t rgb = (uint32_t)pix->z << 16 | (uint32_t)pix->y << 8 | (uint32_t)pix->x;
                color_pt.rgb = *(float *)(&rgb);
                cloud_colored->push_back(color_pt);
            }

            //ROS_INFO_STREAM(color_pt);
        }
    }

    //cv::Matx34d projection_matrix=cam_model_.fullProjectionMatrix();
    //cam_model_.project3dToPixel(cv::Point3d(-0.1392072,-0.02571392, 2.50376511) );
    //b = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB> > (topic, queue_size);
    //pub.publish(cloud)

    //cloud->header.frame_id = image->header.frame_id;
    cloud_colored->header = cloud_out->header;
    point_cloud_pub.publish(cloud_colored);
}

void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr &info_msg)
{

    cam_model_.fromCameraInfo(info_msg);
    cam_model_flag = true;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "myproject1_node");
    ros::NodeHandle nh;

    //todo time sync filter subscriber

    message_filters::Subscriber<sensor_msgs::Image> image_sub(nh, "/sensor/camera/blackfly/image_rect_color", 1000);
    message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub(nh, "/sensor/laser/fx8/point_cloud", 1000);

    typedef sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2> MySyncPolicy;

    if (cam_model_flag == false)
    {
        sub = nh.subscribe("/sensor/camera/blackfly/camera_info", 100, cameraInfoCallback);
    }

    Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), image_sub, cloud_sub);
    sync.registerCallback(boost::bind(&callback, _1, _2));

    point_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("point_cloud_colored", 1, true);

    //ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("/sensor/laser/fx8/point_cloud", 10, callback);

    /* void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& info_msg)
{

    cam_model_.fromCameraInfo(info_msg);

wait for cam info , only do it once

    cv::Matx34d projection_matrix=cam_model_.fullProjectionMatrix();
    cam_model_.project3dToPixel(cv::Point3d(-0.1392072,-0.02571392, 2.50376511) );
//iterate over the point cloud
}
*/

    //tf::poseTFToMsg(transform);
    //ROS_INFO_STREAM("Hello, yall!");
    ros::spin();
}
