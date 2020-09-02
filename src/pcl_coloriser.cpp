#include "myproject1/pcl_coloriser.h"

#include <geometry_msgs/TransformStamped.h>
#include <pcl_conversions/pcl_conversions.h> // pcl::concatenate
#include <tf2_sensor_msgs/tf2_sensor_msgs.h> // tf2::doTransform for PointCloud2

PclColoriser::PclColoriser(std::string fixed_frame, tf2_ros::Buffer *tf_buffer, int max_buffer_size) : PclIntegrator(fixed_frame, Window(fixed_frame), tf_buffer, max_buffer_size)
{
}

bool PclIntegrator::integrate(const sensor_msgs::PointCloud2::ConstPtr &pcl_msg_orig)
{
    sensor_msgs::PointCloud2::Ptr pcl_msg(new sensor_msgs::PointCloud2);
    *pcl_msg = *pcl_msg_orig;
    if (!cloud_buffer_.empty() && cloud_buffer_.back()->header.stamp == pcl_msg->header.stamp)
    {
        // ignore same point cloud (timestamp wise)
        ROS_ERROR("%s: same timestamp pcl or cloud buffer empty. %s ---- %s", __func__, std::to_string(cloud_buffer_.back()->header.stamp.toNSec()).c_str(), std::to_string(pcl_msg->header.stamp.toNSec()).c_str());
        return false;
    }
    geometry_msgs::TransformStamped transformStamped;
    try
    {
        transformStamped = tf_buffer_->lookupTransform(fixed_frame_, pcl_msg->header.frame_id, pcl_msg->header.stamp);
    }
    catch (tf2::TransformException &ex)
    {
        ROS_ERROR("%s", ex.what());
        ROS_ERROR("%s: cannot transform incoming pcl to fixed frame %s.", __func__, fixed_frame_.c_str());
        return false;
    }
    tf2::doTransform(*pcl_msg, *pcl_msg, transformStamped);
    ROS_ERROR("transforming and pushing %s", std::to_string(pcl_msg->header.stamp.toNSec()).c_str());
    cloud_buffer_.push_back(pcl_msg);

    int buffer_size = std::max(max_buffer_size_, 1);
    while (cloud_buffer_.size() > buffer_size)
    {
        cloud_buffer_.pop_front();
    }
    if (cloud_buffer_.empty())
    {
        ROS_ERROR("%s: cloud buffer empty!", __func__);
        return false;
    }
    return true;
}

sensor_msgs::PointCloud2::Ptr PclIntegrator::colorisedCloud(const sensor_msgs::ImageConstPtr &image, const sensor_msgs::PointCloud2ConstPtr &pcl_msg) const
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
                // color_pt.intensity = it->intensity;
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



void publishable_point(float pt)
{
    std::vector<uint8_t> point_binary;
    point_binary = floattoeight(pt);
    for (int j = 0; j < 4; j++)
    {
        point_data.push_back(point_binary[j]);
    }
}

void set_pcl_fields()
{
    //global pt_field and fields
    pt_field.name = 'x';
    pt_field.offset = 0;
    pt_field.datatype = pt_field.FLOAT32;
    pt_field.count = 1;
    fields.push_back(pt_field);
    pt_field.name = 'y';
    pt_field.offset = 4;
    pt_field.datatype = pt_field.FLOAT32;
    pt_field.count = 1;
    fields.push_back(pt_field);
    pt_field.name = 'z';
    pt_field.offset = 8;
    pt_field.datatype = pt_field.FLOAT32;
    pt_field.count = 1;
    fields.push_back(pt_field);
    pt_field.name = "intensity";
    pt_field.offset = 12;
    pt_field.datatype = pt_field.FLOAT32;
    pt_field.count = 1;
    fields.push_back(pt_field);
    pt_field.name = "rgba";
    pt_field.offset = 16;
    pt_field.datatype = pt_field.UINT32;
    pt_field.count = 1;
    fields.push_back(pt_field);
    

}

bool point_valid(int cc, int rr)
{
    int point_indice = (rr)*cloud_in->width + (cc);
    if ((point_indice <= cloud_in->points.size() - 1) && (point_indice >= 0))
    {
        if (isfinite(cloud_in->at(cc, rr).x) && isfinite(cloud_in->at(cc, rr).y) && isfinite(cloud_in->at(cc, rr).z))
        {
            return true;
        }
    }
    return false;
}

std::vector<uint8_t> floattoeight(float argn)
{
    std::vector<uint8_t> point_bin(4);
    union {
        float f;
        struct
        {
            uint8_t ut0, ut1, ut2, ut3;
        } bytes;
    } converter = {.f = argn};
    //std::cout << "float" << argn << "bin" << unsigned(converter.bytes.ut1) << unsigned(converter.bytes.ut2) << unsigned(converter.bytes.ut3) << unsigned(converter.bytes.ut4) << std::endl;
    point_bin[0] = converter.bytes.ut0;
    point_bin[1] = converter.bytes.ut1;
    point_bin[2] = converter.bytes.ut2;
    point_bin[3] = converter.bytes.ut3;
    return point_bin;
}
