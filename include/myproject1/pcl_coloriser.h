#ifndef _PCL_COLORISER_H_
#define _PCL_COLORISER_H_

#include <tf2_ros/buffer.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/image_encodings.h>

#include <geometry_msgs/TransformStamped.h>
#include <pcl_conversions/pcl_conversions.h> // pcl::concatenate
#include <tf2_sensor_msgs/tf2_sensor_msgs.h> // tf2::doTransform for PointCloud2
#include <sensor_msgs/Image.h>

#include <opencv/cv.h>
#include <cv_bridge/cv_bridge.h>


class PclColoriser
{
public:

    PclColoriser( tf2_ros::Buffer *tf_buffer);
    bool setCameraInfo(sensor_msgs::CameraInfo cam_info_msg);
    sensor_msgs::PointCloud2::Ptr colorisedCloud(const sensor_msgs::ImageConstPtr &image, const sensor_msgs::PointCloud2ConstPtr &pcl_msg) const;

protected:
    // sensor_msgs::CameraInfo cam_info_msg_;
    tf2_ros::Buffer* tf_buffer_;
    image_geometry::PinholeCameraModel cam_model_;
    bool cam_info_flag;

    // helper functions
    // sensor_msgs::PointCloud2::Ptr crop_pcl(const sensor_msgs::PointCloud2::Ptr &pcl) const;
};

#endif // _PCL_COLORISER_H_
