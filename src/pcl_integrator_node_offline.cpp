#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <tf2_msgs/TFMessage.h>
#include <cxxopts.hpp>


#include "myproject1/pcl_integrator.h"

bool fillTfBufferFromBag(rosbag::View& view, ros::Duration bagDuration ,tf2_ros::Buffer* tfBuffer);

int main(int argc, char *argv[])
{
    cxxopts::Options options("pcl_integrator_offline", "pcl_integrator_offline_node");

    options.add_options()
        ("f,fixed_frame", "fixed frame for integration", cxxopts::value<std::string>()->default_value("odom"))
        ("b,crop_frame", "base footprint frame to crop", cxxopts::value<std::string>()->default_value("base_footprint"))
        ("i,input_bag_path", "bag for input bag with raw pcls", cxxopts::value<std::string>()->default_value("/home/srbh/agrirobo_proj/with_pcls/largeplants.bag"))
        ("t,pcl_topic", "the pcl topic to integrate", cxxopts::value<std::string>()->default_value("/sensor/laser/vlp16/front/pointcloud_xyzi"))
        ("c,crop_flag", "Enable cropping pcl", cxxopts::value<bool>()->default_value("true"))
        ("m,max_buffer_size", "Param maximum buffer size", cxxopts::value<int>()->default_value("10"))
        ("x,size_x", "Param x for crop box in m", cxxopts::value<float>()->default_value("8."))
        ("y,size_y", "Param y for crop box in m", cxxopts::value<float>()->default_value("6."))
        ("w,offset_x", "Param x offset for crop box in m", cxxopts::value<float>()->default_value("1."))
        ("z,offset_y", "Param y offset for crop box in m", cxxopts::value<float>()->default_value("0."))
        ("h,help", "Print usage")
    ;

    auto result = options.parse(argc, argv);

    if (result.count("help"))
    {
      std::cout << options.help() << std::endl;
      exit(0);
    }
    std::string fixed_frame;
    if (result.count("fixed_frame"))
    {
        fixed_frame = result["fixed_frame"].as<std::string>();
    }
    std::string crop_frame;
    if (result.count("crop_frame"))
    {
        crop_frame = result["crop_frame"].as<std::string>();
    }
    std::string input_bag_path;
    if (result.count("input_bag_path"))
    {
        input_bag_path = result["input_bag_path"].as<std::string>();
    }
    std::string pcl_topic;
    if (result.count("pcl_topic"))
    {
        pcl_topic = result["pcl_topic"].as<std::string>();
    }
      
    bool crop_flag = result["crop_flag"].as<bool>();
    int max_buffer_size = result["max_buffer_size"].as<int>();
    float size_x = result["size_x"].as<float>();
    float size_y = result["size_y"].as<float>();
    float offset_x = result["offset_x"].as<float>();
    float offset_y = result["offset_y"].as<float>();
    std::string target_frame = crop_frame;
/*
    bool crop_flag = true;
    int max_buffer_size = 10;
    float size_x = 8.;
    float size_y = 6.;
    float offset_x = 1.;
    float offset_y = 0.;
    std::string fixed_frame = "odom";
    std::string base_footprint = "base_footprint";
    std::string target_frame = base_footprint;
    std::string input_bag_path = "/home/srbh/agrirobo_proj/with_pcls/largeplants.bag";
    std::string pcl_topic = "/sensor/laser/vlp16/front/pointcloud_xyzi"; 
    
*/

    // initialize ros
    rosbag::Bag input_bag, output_bag;
    input_bag.open(input_bag_path, rosbag::bagmode::Read);
    output_bag.open(input_bag_path+"_integrated.bag", rosbag::bagmode::Write);

    
    
    std::vector<std::string> topics;
    topics.push_back("/tf");
    rosbag::View view(input_bag, rosbag::TopicQuery(topics));
    ros::Time::init();
    ros::Time start, end;
    start = view.getBeginTime();
    end = view.getEndTime();

    ros::Duration bagDuration = end - start;
    bagDuration *= 1.5;
    tf2_ros::Buffer tf_buffer(bagDuration);
    
    if(!fillTfBufferFromBag(view,bagDuration, &tf_buffer)){
        return 1;
    }

    Window window(crop_frame, size_x, size_y, offset_x, offset_y);
    PclIntegrator pcl_integrator(fixed_frame, window, &tf_buffer, max_buffer_size);
    
    rosbag::View view_all(input_bag);
    
    for (rosbag::MessageInstance const msg : view_all)
    {
        sensor_msgs::PointCloud2::ConstPtr pcl_msg = msg.instantiate<sensor_msgs::PointCloud2>();
        if (pcl_msg == NULL)
        {
            // not a pcl msg -> write to bag
            output_bag.write(msg.getTopic(), msg.getTime(), msg);
            continue;
        }
        if(!msg.getTopic().compare(pcl_topic) == 0){
            // not the right pcl topic -> write to bag
            output_bag.write(msg.getTopic(), msg.getTime(), msg);
            continue;
        }

        if(!pcl_integrator.integrate(pcl_msg)){
            ROS_ERROR("%s: integrating pcl at time %.9f.", __func__, pcl_msg->header.stamp.toSec());
            continue;
        }
        sensor_msgs::PointCloud2::Ptr integrated_pcl = pcl_integrator.integratedCloud(target_frame, crop_flag);
        output_bag.write(pcl_topic+"_integrated", msg.getTime(), integrated_pcl);
    }
    input_bag.close();
    output_bag.close();
    std::cout << "done" << std::endl;
    return 0;
}

bool fillTfBufferFromBag(rosbag::View& view, ros::Duration bagDuration ,tf2_ros::Buffer* tfBuffer)
{
    if(bagDuration.toSec() < 1e-3){
        ROS_ERROR("input bag is empty!!!");
        return false;
    }
       
    tfBuffer->setUsingDedicatedThread(true);

    // fill tf buffer
    std::cout << "filling tf buffer..." << std::endl;
    rosbag::View::const_iterator it;
    int tf_count = 0;
    for(it = view.begin(); it != view.end(); ++it){
        if(it->isType<tf2_msgs::TFMessage>()){
            ROS_INFO_THROTTLE(0.5, "t");
            tf2_msgs::TFMessage::Ptr tfMessage = it->instantiate<tf2_msgs::TFMessage>();
            // update tfBuffer
            for(size_t i = 0; i < tfMessage->transforms.size(); ++i){
                if(!tfBuffer->setTransform(tfMessage->transforms[i], "bag_file")){
                    std::cout << "x" << std::flush;
                }
            }
            tf_count++;
            continue;
        }
    }
    std::cout << std::endl << "parsed " << tf_count << " tf messages." << std::endl;
    return true;
}
