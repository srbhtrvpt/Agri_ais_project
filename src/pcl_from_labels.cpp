
bool get_labels(std::string pcl_name)
{
    FILE *output;
    std::string cmd, flpath;
    flpath = "/home/srbh/catkin_ws/src/myproject1/scripts/pcl_get_labels.py " + pcl_name;
    cmd = "python3 " + flpath;
    // output = popen (cmd.c_str(), "r");
    // if (!output){
    //   ROS_ERROR("incorrect parameters or too many files");
    //   return false;
    // }
    // if (pclose (output) != 0){
    //     ROS_ERROR("Could not run file");
    //     return false;
    // }
    try
    {
        system(cmd.c_str());
        return true;
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
        return false;
    }
}

std::vector<std::string> *open_file(std::string pcl_name)
{
    std::string fname;
    std::vector<std::string> *labels = new std::vector<std::string>();
    fname = filepath + "labels_" + pcl_name + ".txt";
    std::ifstream in(fname.c_str());
    if (!in)
    {
        return NULL;
    }
    std::string str;
    while (std::getline(in, str))
    {
        if (str.size() > 0)
            labels->push_back(str);
    }
    in.close();
    return labels;
}

sensor_msgs::PointCloud2::Ptr add_labels(sensor_msgs::PointCloud2::Ptr pcl_msg)
{

    std::vector<std::string> *labels = open_file(std::to_string(pcl_msg->header.stamp.toNSec()));
    if (!labels)
    {
        return NULL;
    }
    else
    {
        sensor_msgs::PointCloud2::Ptr labelled_pcl(new sensor_msgs::PointCloud2);
        sensor_msgs::PointCloud2Iterator<float> it_x(*pcl_msg, "x");
        sensor_msgs::PointCloud2Iterator<float> it_y(*pcl_msg, "y");
        sensor_msgs::PointCloud2Iterator<float> it_z(*pcl_msg, "z");
        sensor_msgs::PointCloud2Iterator<float> it_int(*pcl_msg, "intensity");
        sensor_msgs::PointCloud2Iterator<float> it_rgb(*pcl_msg, "rgb");
        sensor_msgs::PointCloud2Iterator<float> it_tgi(*pcl_msg, "tgi");
        sensor_msgs::PointCloud2Iterator<float> it_vari(*pcl_msg, "vari");
        sensor_msgs::PointCloud2Iterator<float> it_curv(*pcl_msg, "curvature");

        labelled_pcl->is_bigendian = false;
        labelled_pcl->is_dense = false;
        sensor_msgs::PointCloud2Modifier modifier(*labelled_pcl);
        modifier.setPointCloud2Fields(9, "x", 1, sensor_msgs::PointField::FLOAT32,
                                      "y", 1, sensor_msgs::PointField::FLOAT32,
                                      "z", 1, sensor_msgs::PointField::FLOAT32,
                                      "intensity", 1, sensor_msgs::PointField::FLOAT32,
                                      "tgi", 1, sensor_msgs::PointField::FLOAT32,
                                      "rgb", 1, sensor_msgs::PointField::FLOAT32,
                                      "vari", 1, sensor_msgs::PointField::FLOAT32,
                                      "label", 1, sensor_msgs::PointField::FLOAT32,
                                      "curvature", 1, sensor_msgs::PointField::FLOAT32);
        modifier.resize(pcl_msg->width);
        sensor_msgs::PointCloud2Iterator<float> out_x(*labelled_pcl, "x");
        sensor_msgs::PointCloud2Iterator<float> out_y(*labelled_pcl, "y");
        sensor_msgs::PointCloud2Iterator<float> out_z(*labelled_pcl, "z");
        sensor_msgs::PointCloud2Iterator<float> out_int(*labelled_pcl, "intensity");
        sensor_msgs::PointCloud2Iterator<float> out_rgb(*labelled_pcl, "rgb");
        sensor_msgs::PointCloud2Iterator<float> out_tgi(*labelled_pcl, "tgi");
        sensor_msgs::PointCloud2Iterator<float> out_vari(*labelled_pcl, "vari");
        sensor_msgs::PointCloud2Iterator<float> out_curv(*labelled_pcl, "curvature");
        sensor_msgs::PointCloud2Iterator<float> out_label(*labelled_pcl, "label");

        int numpoints = 0;
        for (std::vector<std::string>::iterator it = std::begin(*labels); it != std::end(*labels); ++it)
        {
            *out_x = *it_x;
            *out_y = *it_y;
            *out_z = *it_z;
            *out_curv = *it_curv;
            *out_rgb = *it_rgb;
            *out_tgi = *it_tgi;
            *out_vari = *it_vari;
            *out_int = *it_int;
            *out_label = std::stof(*it);
            ++out_x;
            ++out_y;
            ++out_z;
            ++out_int;
            ++out_rgb;
            ++out_tgi;
            ++out_curv;
            ++out_vari;
            ++out_label;
            numpoints++;
            ++it_rgb;
            ++it_tgi;
            ++it_vari;
            ++it_int;
            ++it_x;
            ++it_y;
            ++it_z;
            ++it_curv;
        }
        modifier.resize(numpoints);
        labelled_pcl->header.frame_id = pcl_msg->header.frame_id;
        labelled_pcl->header.stamp = pcl_msg->header.stamp;

        return labelled_pcl;
    }
}
