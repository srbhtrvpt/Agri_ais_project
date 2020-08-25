#define PCL_NO_PRECOMPILE
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
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
#include <tf2_sensor_msgs/tf2_sensor_msgs.h> // tf2::doTransform for PointCloud2
#include <geometry_msgs/TransformStamped.h>


struct PointXYZIRGB
{
    PCL_ADD_POINT4D;
    PCL_ADD_RGB;
    PCL_ADD_INTENSITY;               
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW     // make sure our new allocators are aligned
} EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRGB,           // here we assume a XYZ (as fields)
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (uint32_t, rgb, rgb)
                                    (float, intensity , intensity)
)

using namespace message_filters;
ros::Publisher point_cloud_pub;
image_geometry::PinholeCameraModel cam_model_;
ros::Subscriber sub;

typedef pcl::PointXYZI PointT;
typedef pcl::PointXYZRGB PointC;
// typedef pcl::PointCloud<PointC> ColoredPointCloud;

typedef pcl::PointCloud<PointXYZIRGB> ColoredPointCloud;
typedef pcl::PointCloud<PointT> PointCloud;

bool setCameraInfo(const sensor_msgs::CameraInfoConstPtr &cam_info_msg)
{
    try
    {
        cam_model_.fromCameraInfo(cam_info_msg);
        return true;
    }
    catch (image_geometry::Exception ex)
    {
        ROS_ERROR("%s", ex.what());
        return false;
    }
}

void callback(const sensor_msgs::ImageConstPtr &image, const sensor_msgs::PointCloud2ConstPtr &cloud)
{
    PointCloud::Ptr cloud_in(new PointCloud);
    ColoredPointCloud::Ptr cloud_colored(new ColoredPointCloud);
    PointCloud::iterator it;
    // PointC color_pt;
    PointXYZIRGB color_pt; 
    tf::TransformListener listener;
    tf::StampedTransform transform;
    cv::Point2d uv;
    cv::Mat image_in;
    cv_bridge::CvImagePtr input_bridge;
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener(tf_buffer); 
    

    try
    {
        input_bridge = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
        image_in = input_bridge->image;
    }
    catch (cv_bridge::Exception &ex)
    {
        ROS_ERROR("Failed to convert image");
    }
    pcl::fromROSMsg(*cloud, *cloud_in);

    try
    {
        listener.waitForTransform(image->header.frame_id, cloud->header.frame_id, ros::Time(0), ros::Duration(5.0));
        listener.lookupTransform(image->header.frame_id, cloud->header.frame_id, ros::Time(0), transform);
        pcl_ros::transformPointCloud(*cloud_in, *cloud_in, transform);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s", ex.what());
    }

    if (setCameraInfo(ros::topic::waitForMessage<sensor_msgs::CameraInfo>("camera_info", ros::Duration(1.0))))
    {
        for (it = cloud_in->points.begin(); it < cloud_in->points.end(); it++)
        {
            cv::Point3d pt_cv(it->x, it->y, it->z);
            uv = cam_model_.project3dToPixel(pt_cv);
            if (uv.x > 0 && uv.x < image_in.cols && uv.y < image_in.rows && uv.y > 0)
            {
                cv::Point3_<uchar> *pix = image_in.ptr<cv::Point3_<uchar>>(uv.y, uv.x);
                color_pt.x = it->x;
                color_pt.y = it->y;
                color_pt.z = it->z;
                uint32_t rgb = (uint32_t)pix->z << 16 | (uint32_t)pix->y << 8 | (uint32_t)pix->x;
                color_pt.rgb = *(float *)(&rgb);
                color_pt.intensity = it->intensity;
                // cloud_colored.push_back(color_pt);
                cloud_colored->push_back(color_pt);

            }
        }
    }
    try
    {
        // pcl_ros::transformPointCloud(*cloud_colored, *cloud_colored, transform.inverse());


    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s", ex.what());
    }
    // cloud_out->header = cloud->header;
    // point_cloud_pub.publish(cloud_out);

    cloud_colored->header = cloud_in->header;
    point_cloud_pub.publish(cloud_colored);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "pcl_colored_node");
    ros::NodeHandle nh;

    message_filters::Subscriber<sensor_msgs::Image> image_sub(nh, "input_image", 10);
    message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub(nh, "input_cloud", 10);

    typedef sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2> MySyncPolicy;

    Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), image_sub, cloud_sub);
    sync.registerCallback(boost::bind(&callback, _1, _2));

    point_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("output_cloud", 10, true);

    ros::spin();
}