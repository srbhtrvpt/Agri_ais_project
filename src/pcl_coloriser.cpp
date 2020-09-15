#include "myproject1/pcl_coloriser.h"

#include <geometry_msgs/TransformStamped.h>
#include <pcl_conversions/pcl_conversions.h> // pcl::concatenate
#include <tf2_sensor_msgs/tf2_sensor_msgs.h> // tf2::doTransform for PointCloud2
#include <sensor_msgs/Image.h>

#include <opencv/cv.h>
#include <cv_bridge/cv_bridge.h>



PclColoriser::PclColoriser(tf2_ros::Buffer *tf_buffer): tf_buffer_(tf_buffer){
}

bool PclColoriser::setCameraInfo(sensor_msgs::CameraInfo cam_info_msg){
//     try
//     {
//         cam_model_.fromCameraInfo(cam_info_msg);
//         return true;
//     }
//     catch (image_geometry::Exception ex)
//     {
//         ROS_ERROR("%s", ex.what());
//         return false;
//     }
    return cam_model_.fromCameraInfo(cam_info_msg);

 }

sensor_msgs::PointCloud2::Ptr PclColoriser::colorisedCloud(const sensor_msgs::ImageConstPtr &image, const sensor_msgs::PointCloud2ConstPtr &pcl_msg_orig) const
{
    sensor_msgs::PointCloud2::Ptr pcl_msg(new sensor_msgs::PointCloud2), cloud_colored(new sensor_msgs::PointCloud2);
    *pcl_msg = *pcl_msg_orig;    
    geometry_msgs::TransformStamped transformStamped;
    cv::Point2d uv;
    cv::Mat image_in;
    cv_bridge::CvImagePtr input_bridge;

    try
    {
        input_bridge = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
        image_in = input_bridge->image;
    }
    catch (cv_bridge::Exception &ex)
    {
        ROS_ERROR("Failed to convert image");
    }

    try
    {
        transformStamped = tf_buffer_->lookupTransform(image->header.frame_id, pcl_msg->header.frame_id, ros::Time(0),ros::Duration(5.0));
    }
    catch (tf2::TransformException &ex)
    {
        ROS_ERROR("%s", ex.what());       
    }
    tf2::doTransform(*pcl_msg, *pcl_msg, transformStamped);

    sensor_msgs::PointCloud2Iterator<float> it_x(*pcl_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> it_y(*pcl_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> it_z(*pcl_msg, "z");
    sensor_msgs::PointCloud2Iterator<float> it_int(*pcl_msg, "intensity");

    // cloud_colored->width  = pcl_msg->width;
    // cloud_colored->height = 1;
    cloud_colored->is_bigendian = false;
    cloud_colored->is_dense = false;
    sensor_msgs::PointCloud2Modifier modifier(*cloud_colored);
    modifier.setPointCloud2Fields( 6, "x", 1, sensor_msgs::PointField::FLOAT32,
                                        "y", 1, sensor_msgs::PointField::FLOAT32,
                                        "z", 1, sensor_msgs::PointField::FLOAT32,
                                        "intensity", 1, sensor_msgs::PointField::FLOAT32,
                                        "tgi", 1, sensor_msgs::PointField::FLOAT32,
                                        "rgb", 1, sensor_msgs::PointField::FLOAT32);
    modifier.resize(pcl_msg->width);
    sensor_msgs::PointCloud2Iterator<float> out_x(*cloud_colored, "x");
    sensor_msgs::PointCloud2Iterator<float> out_y(*cloud_colored, "y");
    sensor_msgs::PointCloud2Iterator<float> out_z(*cloud_colored, "z");
    sensor_msgs::PointCloud2Iterator<float> out_int(*cloud_colored, "intensity");
    sensor_msgs::PointCloud2Iterator<float> out_rgb(*cloud_colored, "rgb");
    sensor_msgs::PointCloud2Iterator<float> out_tgi(*cloud_colored, "tgi");

    int numpoints = 0;
    double x,y,z;
    while (it_x != it_x.end())
    {   
        x = *it_x;
        y = *it_y;
        z = *it_z;
        cv::Point3d pt_cv(x, y, z);
        uv = cam_model_.project3dToPixel(pt_cv);
        if (uv.x > 0 && uv.x < image_in.cols && uv.y < image_in.rows && uv.y > 0)
        {
            *out_x = *it_x;
            *out_y = *it_x;
            *out_z = *it_x;
            cv::Point3_<uchar> *pix = image_in.ptr<cv::Point3_<uchar>>(uv.y, uv.x);
            uint32_t rgb = (uint32_t)pix->z << 16 | (uint32_t)pix->y << 8 | (uint32_t)pix->x;
            *out_rgb = *(float *)(&rgb);
            *out_int = *it_int;
            float red = pix->z;
            float green = pix->y;
            float blue = pix->x;
            float TGI = -0.5*(190*(red - green) - 120*(red - blue));
            *out_tgi = TGI;
            ++out_x;
            ++out_y;
            ++out_z;
            ++out_int;
            ++out_rgb;
            ++out_tgi;
            numpoints++;
        }
        ++it_x;
        ++it_y;
        ++it_z;
        ++it_int;
    }
    modifier.resize(numpoints);
    try
    {
        transformStamped = tf_buffer_->lookupTransform(pcl_msg->header.frame_id,image->header.frame_id, ros::Time(0));
    }
    catch (tf2::TransformException &ex)
    {
        ROS_ERROR("%s", ex.what());       
    }
    tf2::doTransform(*cloud_colored, *cloud_colored, transformStamped);
    cloud_colored->header.stamp = pcl_msg->header.stamp;
    return cloud_colored;
}
