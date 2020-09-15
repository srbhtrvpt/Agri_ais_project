#include "myproject1/pcl_segmentor.h"




PclSegmentor::PclSegmentor(const sensor_msgs::PointCloud2ConstPtr &pcl_msg_orig){
    *pcl_msg = *pcl_msg_orig;
}


sensor_msgs::PointCloud2::Ptr PclSegmentor::curvatureCloud(int KSearchRadius){

    sensor_msgs::PointCloud2::Ptr curvature_pcl(new sensor_msgs::PointCloud2);
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
    std::vector<int> map;
    pcl::NormalEstimation<PointT, PointNT> ne;

    pcl::fromROSMsg(*pcl_msg, *cloud_in);
    cloud_in->is_dense = false;
    pcl::removeNaNFromPointCloud(*cloud_in, *cloud_in, map);
    ne.setInputCloud(cloud_in);
    ne.setSearchMethod(tree);
    // ne.setRadiusSearch(0.3);
    ne.setKSearch(KSearchRadius);
    try
    {
       ne.compute(*cloud_normals);
        computed_normals = true;
    }
    catch(const std::exception& e)
    {
        ROS_ERROR("Failed to compute normals");
        computed_normals = false;

    }
    
    sensor_msgs::PointCloud2Iterator<float> it_tgi(*pcl_msg, "tgi");
    sensor_msgs::PointCloud2Iterator<float> it_rgb(*pcl_msg, "rgb");

    
    curvature_pcl->is_bigendian = false;
    curvature_pcl->is_dense = false;
    sensor_msgs::PointCloud2Modifier modifier(*curvature_pcl);
    modifier.setPointCloud2Fields( 7, "x", 1, sensor_msgs::PointField::FLOAT32,
                                        "y", 1, sensor_msgs::PointField::FLOAT32,
                                        "z", 1, sensor_msgs::PointField::FLOAT32,
                                        "intensity", 1, sensor_msgs::PointField::FLOAT32,
                                        "tgi", 1, sensor_msgs::PointField::FLOAT32,
                                        "rgb", 1, sensor_msgs::PointField::FLOAT32,
                                        "curvature", 1, sensor_msgs::PointField::FLOAT32);
    modifier.resize(pcl_msg->width);
    sensor_msgs::PointCloud2Iterator<float> out_x(*curvature_pcl, "x");
    sensor_msgs::PointCloud2Iterator<float> out_y(*curvature_pcl, "y");
    sensor_msgs::PointCloud2Iterator<float> out_z(*curvature_pcl, "z");
    sensor_msgs::PointCloud2Iterator<float> out_int(*curvature_pcl, "intensity");
    sensor_msgs::PointCloud2Iterator<float> out_rgb(*curvature_pcl, "rgb");
    sensor_msgs::PointCloud2Iterator<float> out_tgi(*curvature_pcl, "tgi");
    sensor_msgs::PointCloud2Iterator<float> out_curv(*curvature_pcl, "curvature");

    int numpoints = 0;
    for (int i = 0; i < cloud_normals->points.size(); i++){
        if(isfinite(cloud_normals->points[i].curvature)){
            *out_x = cloud_in->points[i].x;
            *out_y = cloud_in->points[i].y;
            *out_z= cloud_in->points[i].z;
            *out_int = cloud_in->points[i].intensity;
            *out_curv = cloud_normals->points[i].curvature;
            *out_rgb = *it_rgb;
            *out_tgi = *it_tgi;
            ++out_x;
            ++out_y;
            ++out_z;
            ++out_int;
            ++out_rgb;
            ++out_tgi;
            ++out_curv;
            numpoints++;   
        }
        ++it_rgb;
        ++it_tgi;
    }
    modifier.resize(numpoints);
    curvature_pcl->header = pcl_msg->header;
    return curvature_pcl;
}

PointCloudN::Ptr PclSegmentor::getCloudNormals(){
    if(!computed_normals){
        return NULL;
    }
    else{
        return cloud_normals;
    }
}

bool PclSegmentor::segmentPcl(double NormalDistanceWeight, double DistanceThreshold){

    pcl::SACSegmentationFromNormals<PointT, PointNT> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);
    seg.setNormalDistanceWeight(NormalDistanceWeight);
    seg.setDistanceThreshold(DistanceThreshold);

    // plane model
    /*
    coefficients->values.resize(4);
    pcl::SACSegmentation<PointT> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setDistanceThreshold(DistanceThreshold); */

    seg.setInputCloud(cloud_in);
    seg.setInputNormals(cloud_normals);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.segment(*inliers, *coefficients);
    if (inliers->indices.size() == 0)
    {
        PCL_ERROR("Could not estimate a planar model.\n");
        segmented = false;
        return segmented;
    }
    //extract
    else
    {
        segmented = true;
        return segmented;
    }
}

sensor_msgs::PointCloud2::Ptr PclSegmentor::segmentedCloud(){

    if(!segmented){
        return NULL;
    }
    else{
        PointCloud::Ptr cloud_inliers(new PointCloud);
        sensor_msgs::PointCloud2::Ptr inliers_pcl(new sensor_msgs::PointCloud2);
        pcl::ExtractIndices<PointT> extract;
        extract.setInputCloud(cloud_in);
        extract.setIndices(inliers);
        extract.setNegative(false);     // Extract the inliers
        extract.filter(*cloud_inliers); // cloud_inliers contains the plane
        pcl::toROSMsg(*cloud_inliers, *inliers_pcl);
        return inliers_pcl;

        // extract.setNegative(true); // Extract the outliers
        // extract.filter(*cloud_outliers);
    }
}


pcl::ModelCoefficients::Ptr PclSegmentor::getCoefficients(){
    if(!segmented){
        return NULL;
    }
    else{
        return coefficients;
    }
}

pcl::PointIndices::Ptr PclSegmentor::getInlierIndices(){
    if(!segmented){
        return NULL;
    }
    else{
        return inliers;
    }
}