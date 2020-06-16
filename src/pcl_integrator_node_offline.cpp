#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <tf2_msgs/TFMessage.h>

#include "pcl_integrator.h"

bool fillTfBufferFromBag(rosbag::View& view, tf2_ros::Buffer* tfBuffer);

int main(int argc, char *argv[])
{
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
    
    // TODO: read paramters from command line (use argparser and argv)!!!
    //target_frame = base_footprint;

    // initialize ros
    rosbag::Bag input_bag, output_bag;
    input_bag.open(input_bag_path, rosbag::bagmode::Read);
    output_bag.open(input_bag_path+"_integrated.bag", rosbag::bagmode::Write);

    tf2_ros::Buffer tf_buffer;
    std::vector<std::string> topics;
    topics.push_back("/tf");
    rosbag::View view(input_bag, rosbag::TopicQuery(topics));
    if(!fillTfBufferFromBag(view, &tf_buffer)){
        return 1;
    }

    Window window(base_footprint, size_x, size_y, offset_x, offset_y);
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

bool fillTfBufferFromBag(rosbag::View& view, tf2_ros::Buffer* tfBuffer)
{
    ros::Time start, end;
    start = view.getBeginTime();
    end = view.getEndTime();

    ros::Duration bagDuration = end - start;
    bagDuration *= 1.5;

    if(bagDuration.toSec() < 1e-3){
        ROS_ERROR("input bag is empty!!!");
        return false;
    }

    // TODO: set buffer size??
    
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
