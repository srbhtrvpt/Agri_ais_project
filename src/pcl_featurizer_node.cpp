#include <tf2_ros/transform_listener.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <boost/bind.hpp>
#include <fstream>
#include "myproject1/pcl_coloriser.h" 
#include "myproject1/pcl_segmentor.h"
#include <stdlib.h>

using namespace message_filters;
ros::Publisher point_cloud_pub;
ros::Subscriber sub;
std::string filepath;
bool get_curv, save_flag;
PclColoriser* pcl_coloriser;
sensor_msgs::CameraInfo cam_info_msg;
PclSegmentor* pcl_segmentor;
int ksearch_radius;
tf2_ros::Buffer tf_buffer;


bool write_to_file(std::string filep, std::string data){

    std::ofstream file;
    file.open(filep, std::ios::out | std::ios::app);
    if (file.fail())
    {
        throw std::ios_base::failure(std::strerror(errno));
        return false;
    }
    file.exceptions(file.exceptions() | std::ios::failbit | std::ifstream::badbit);
    file << data << std::endl;
    ROS_INFO_THROTTLE(1, "w");
    file.close();
    return true;
}

void save_pcl_data(sensor_msgs::PointCloud2::Ptr cloud_out, bool get_curv){ 
    geometry_msgs::TransformStamped transformStamped;
    std::string filename,string_out;
    try{
        transformStamped = tf_buffer.lookupTransform("base_footprint",cloud_out->header.frame_id, ros::Time(0));
    }
    catch (tf2::TransformException &ex){
        ROS_ERROR("%s", ex.what());       
    }
    tf2::doTransform(*cloud_out, *cloud_out, transformStamped);
    sensor_msgs::PointCloud2Iterator<float> out_x(*cloud_out, "x");
    sensor_msgs::PointCloud2Iterator<float> out_y(*cloud_out, "y");
    sensor_msgs::PointCloud2Iterator<float> out_z(*cloud_out, "z");
    sensor_msgs::PointCloud2Iterator<float> out_int(*cloud_out, "intensity");
    sensor_msgs::PointCloud2Iterator<float> out_rgb(*cloud_out, "rgb");
    sensor_msgs::PointCloud2Iterator<float> out_tgi(*cloud_out, "tgi");
    sensor_msgs::PointCloud2Iterator<float> out_vari(*cloud_out, "vari");
    if(get_curv){
        sensor_msgs::PointCloud2Iterator<float> out_curv(*cloud_out, "curvature");
        filename = filepath + std::to_string(cloud_out->header.stamp.toNSec()) + ".txt";
        while (out_x != out_x.end()){
        
            string_out = std::to_string(*out_x) 
                        + "\t" + std::to_string(*out_y) 
                        + "\t" + std::to_string(*out_z) 
                        + "\t" + std::to_string(*out_int) 
                        + "\t" + std::to_string(*out_rgb)  
                        + "\t" + std::to_string(*out_tgi)  
                        + "\t" + std::to_string(*out_vari) 
                        + "\t" + std::to_string(*out_curv);    
            if (!write_to_file(filename, string_out)){
                ROS_ERROR("%s: error writing value ", __func__);
            }
            ++out_x;
            ++out_y;
            ++out_z;
            ++out_int;
            ++out_rgb;
            ++out_tgi;
            ++out_vari;
            ++out_curv;
        }
        return;
    }
    filename = filepath + std::to_string(cloud_out->header.stamp.toNSec()) + ".txt";
    while (out_x != out_x.end()){
        string_out = std::to_string(*out_x) 
                    + "\t" + std::to_string(*out_y) 
                    + "\t" + std::to_string(*out_z) 
                    + "\t" + std::to_string(*out_int) 
                    + "\t" + std::to_string(*out_rgb)  
                    + "\t" + std::to_string(*out_tgi)  
                    + "\t" + std::to_string(*out_vari);                       
        if (!write_to_file(filename, string_out)){
            ROS_ERROR("%s: error writing value ", __func__);
        }
        ++out_x;
        ++out_y;
        ++out_z;
        ++out_int;
        ++out_rgb;
        ++out_tgi;
        ++out_vari;
    }
    return;
}  

void callback(const sensor_msgs::ImageConstPtr &image, const sensor_msgs::PointCloud2ConstPtr &cloud)
{
    if(!pcl_coloriser->setCameraInfo(cam_info_msg)){
        return;
    }
   sensor_msgs::PointCloud2::Ptr colorised_pcl = pcl_coloriser->colorisedCloud(image, cloud);
   if(get_curv){
        pcl_segmentor = new PclSegmentor(colorised_pcl);
        if(pcl_segmentor->computePclNormals(ksearch_radius)){
            sensor_msgs::PointCloud2::Ptr cloud_curv = pcl_segmentor->curvatureCloud();
            if(save_flag){
                save_pcl_data(cloud_curv,get_curv);
                // if(get_labels(std::to_string(cloud_curv->header.stamp.toNSec()))){
                //     sensor_msgs::PointCloud2::Ptr cloud_labelled = add_labels(cloud_curv);
                //     point_cloud_pub.publish(*cloud_labelled);
                //     return;
                // }
            }
            point_cloud_pub.publish(*cloud_curv);
            return;
        }
   }
    point_cloud_pub.publish(*colorised_pcl);
    // if(save_flag){
    //     save_pcl_data(colorised_pcl,get_curv);
    // }
}

void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr &info_msg)
{
    cam_info_msg = *info_msg;
}



int main(int argc, char *argv[])
{
    ros::init(argc, argv, "pcl_featurizer_node");
    ros::NodeHandle nh;
    ros::NodeHandle nhPriv("~");
    nhPriv.param<std::string>("op_path", filepath, "/home/srbh/agrirobo_proj/with_pcls/data/");
    nhPriv.param("get_curv", get_curv, true);
    nhPriv.param("KSearchRadius", ksearch_radius, 16);
    nhPriv.param("save_fl", save_flag, false);

    tf2_ros::TransformListener tf_listener(tf_buffer);
    sub = nh.subscribe("camera_info", 5, cameraInfoCallback);
    pcl_coloriser = new PclColoriser(&tf_buffer);


    message_filters::Subscriber<sensor_msgs::Image> image_sub(nh, "input_image", 10);
    message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub(nh, "input_cloud", 10);

    typedef sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2> MySyncPolicy;

    Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), image_sub, cloud_sub);
    sync.registerCallback(boost::bind(&callback, _1, _2));

    point_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("output_cloud", 10, true);



    ros::spin();
    return 0;
}