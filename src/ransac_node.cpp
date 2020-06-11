#include <pcl/ModelCoefficients.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/filter.h>
#include <pcl/io/pcd_io.h>
#include <pcl/pcl_config.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <visualization_msgs/Marker.h>
#include <iostream>
#include <std_msgs/Header.h>

ros::Publisher point_cloud_pub, point_cloud_pub1, point_cloud_pub2, vis_marker_pub, vis_marker_pub1;

typedef pcl::PointXYZ PointT;
typedef pcl::Normal PointNT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointCloud<PointNT> PointCloudN;

std::vector<uint8_t> floattoeight(float argn);
std::vector<int> neighborhood_sq(int cc, int rr);
void publishable_point(float pt);
visualization_msgs::Marker init_normal_marker();
visualization_msgs::Marker init_plane_marker();
void set_pcl_fields();
bool point_valid(int cc, int rr);
std::vector<int> neighborhood_plus(int cc, int rr);

PointCloud::Ptr cloud_in(new PointCloud), cloud_inliers(new PointCloud), cloud_outliers(new PointCloud);
PointCloudN::Ptr cloud_normals(new PointCloudN);
PointCloud::iterator it;
sensor_msgs::PointCloud2::Ptr output_ground(new sensor_msgs::PointCloud2), output_plants(new sensor_msgs::PointCloud2), cloud_out(new sensor_msgs::PointCloud2);

pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
int neighborhood_radius, ksearch_radius; // neigh_radius
double NormalDistanceWeight , DistanceThreshold, curvature_threshold;
std::vector<uint8_t> point_data;
int normal_visualisation_scale;


std_msgs::Header header_cloud_out;
std::vector<sensor_msgs::PointField> fields;
sensor_msgs::PointField pt_field;
std_msgs::ColorRGBA color;
geometry_msgs::Point p;

void publishable_point(float pt)
{   
    std::vector<uint8_t> point_binary;
    point_binary = floattoeight(pt);
    for (int j = 0; j < 4; j++)
    {
        point_data.push_back(point_binary[j]);
    }
}

visualization_msgs::Marker init_normal_marker()
{
    visualization_msgs::Marker normal_marker;
    normal_marker.ns = "normal_markers";
    normal_marker.id = 0;
    normal_marker.type = visualization_msgs::Marker::LINE_LIST;
    normal_marker.action = visualization_msgs::Marker::ADD;
    normal_marker.color.a = 1.0;
    normal_marker.scale.x = 0.01;
    normal_marker.pose.orientation.w = 1.0;

    return normal_marker;
}

visualization_msgs::Marker init_plane_marker()
{
    visualization_msgs::Marker plane_marker;
    plane_marker.header.frame_id = "base_footprint";
    plane_marker.ns = "plane_markers";
    plane_marker.id = 0;
    plane_marker.type = visualization_msgs::Marker::CYLINDER;
    plane_marker.action = visualization_msgs::Marker::ADD;
    plane_marker.pose.position.x = 3;
    plane_marker.pose.position.y = 0;
    plane_marker.pose.position.z = 0;
    plane_marker.scale.x = 2;
    plane_marker.scale.y = 2;
    plane_marker.scale.z = 0.001;
    plane_marker.color.a = 1.0; 
    plane_marker.color.r = 1.0;
    plane_marker.color.g = 0.33;
    plane_marker.color.b = 0.0;
    return plane_marker;
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
    pt_field.name = "curvature";
    pt_field.offset = 12;
    pt_field.datatype = pt_field.FLOAT32;
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

std::vector<int> neighborhood_sq(int cc, int rr)
{
    std::vector<int> result;
    for (int i = -neighborhood_radius; i <= neighborhood_radius; i++)
    {
        for (int j = -neighborhood_radius; j <= neighborhood_radius; j++)
        {
            if (point_valid(cc + j, rr + i))
            {
                result.push_back((rr + i) * cloud_in->width + (cc + j));
            }
        }
    }
    return result;
}

std::vector<int> neighborhood_plus(int cc, int rr)
{
    std::vector<int> result;
    for (int i = -neighborhood_radius; i <= neighborhood_radius; i++)
    {
        if (point_valid(cc, rr + i))
        {
            result.push_back((rr + i) * cloud_in->width + (cc));
        }
    }
    for (int j = -neighborhood_radius; j <= neighborhood_radius; j++)
    {
        if (point_valid(cc + j, rr))
        {
            result.push_back((rr)*cloud_in->width + (cc + j));
        }
    }
    return result;
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

void callback(const sensor_msgs::PointCloud2ConstPtr &cloud_ros)
{
    point_data.clear();
    pcl::fromROSMsg(*cloud_ros, *cloud_in);
    std::vector<int> map;
    cloud_in->is_dense = false;
    pcl::removeNaNFromPointCloud(*cloud_in, *cloud_in, map);
    int counter = 0;
    visualization_msgs::Marker normal_marker = init_normal_marker() , plane_marker = init_plane_marker();

    //normal estimation
    pcl::NormalEstimation<PointT, PointNT> ne;
    ne.setInputCloud(cloud_in);
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
    ne.setSearchMethod(tree);
    // ne.setRadiusSearch(0.3);
    ne.setKSearch(ksearch_radius);
    ne.compute(*cloud_normals);

    for (int i = 0; i < cloud_normals->points.size(); i++)
    {
        if (isfinite(cloud_in->points[i].x) && isfinite(cloud_in->points[i].y) && isfinite(cloud_in->points[i].z) &&
            isfinite(cloud_normals->points[i].normal_z) && isfinite(cloud_normals->points[i].normal_y) &&
            isfinite(cloud_normals->points[i].normal_x) && isfinite(cloud_normals->points[i].curvature))
        {
            counter++;
            if (cloud_normals->points[i].curvature < curvature_threshold)
            {
                color.r = 1.0;
                color.g = 0.0;
                color.b = 0.0;
            }
            else
            {
                color.r = 0.0;
                color.g = 0.0;
                color.b = 1.0;
            }
            p.x = cloud_in->points[i].x;
            p.y = cloud_in->points[i].y;
            p.z = cloud_in->points[i].z;
            normal_marker.points.push_back(p);
            normal_marker.colors.push_back(color);
            p.z += cloud_normals->points[i].normal_z * normal_visualisation_scale * 0.001;
            p.y += cloud_normals->points[i].normal_y * normal_visualisation_scale * 0.001;
            p.x += cloud_normals->points[i].normal_x * normal_visualisation_scale * 0.001;
            normal_marker.points.push_back(p);
            normal_marker.colors.push_back(color);

            publishable_point(cloud_in->points[i].x);
            publishable_point(cloud_in->points[i].y);
            publishable_point(cloud_in->points[i].z);
            publishable_point(cloud_normals->points[i].curvature);
        }
    }
    //std::cout << "count" <<  counter << "size" << cloud_normals->points.size() <<  "points" << points.size() <<  "\n";

    //setup cloud curvature
    header_cloud_out.frame_id = cloud_in->header.frame_id;
    header_cloud_out.stamp = cloud_ros->header.stamp;
    cloud_out->header = header_cloud_out;
    cloud_out->fields = fields;
    cloud_out->data = point_data;
    cloud_out->width = counter;
    cloud_out->row_step = counter;
    cloud_out->point_step = 16; //CHANGE IF MORE FIELDS
    //fixed
    cloud_out->is_bigendian = false;
    cloud_out->height = 1;

    normal_marker.header.frame_id = cloud_ros->header.frame_id;
    normal_marker.header.stamp = cloud_ros->header.stamp;

    point_cloud_pub2.publish(cloud_out);
    vis_marker_pub.publish(normal_marker);

    // plane normal model
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

    plane_marker.pose.orientation.x = coefficients->values[0]/coefficients->values[3];
    plane_marker.pose.orientation.y = coefficients->values[1]/coefficients->values[3];
    plane_marker.pose.orientation.z = coefficients->values[2]/coefficients->values[3];
    plane_marker.pose.orientation.w = 1.0;
    plane_marker.header.stamp = cloud_ros->header.stamp;
    vis_marker_pub1.publish(plane_marker);

    // std::cout << "size"<<inliers->indices.size();
    if (inliers->indices.size() == 0)
    {
        PCL_ERROR("Could not estimate a planar model.\n");
    }
//extract
    else
    {   
        pcl::ExtractIndices<PointT> extract;
        extract.setInputCloud(cloud_in);
        extract.setIndices(inliers);
        extract.setNegative(false); // Extract the inliers
        extract.filter(*cloud_inliers);
        pcl::toROSMsg(*cloud_inliers, *output_ground); // cloud_inliers contains the plane
        point_cloud_pub1.publish(output_ground);

        extract.setNegative(true); // Extract the outliers
        extract.filter(*cloud_outliers);
        pcl::toROSMsg(*cloud_outliers, *output_plants);
        point_cloud_pub.publish(output_plants);
    }
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "ransac_node");
    ros::NodeHandle nh;
    ros::NodeHandle nhPriv("~priv");
    nhPriv.param("neighborhood_radius", neighborhood_radius, 2);
    nhPriv.param("ksearch_radius", ksearch_radius, 16);
    nhPriv.param("normal_visualisation_scale", normal_visualisation_scale, 30);
    nhPriv.param("curvature_threshold", curvature_threshold, 0.00275);
    nhPriv.param("Distance_Threshold", DistanceThreshold, 0.05);
    nhPriv.param("NormalDistanceWeight", NormalDistanceWeight, 0.02);
    set_pcl_fields();
    
    // ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/sensor/laser/vlp16/front/pointcloud_xyzi", 1, callback);
    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("input_cloud", 10, callback);
    vis_marker_pub = nh.advertise<visualization_msgs::Marker>("normal_marker", 1, true);
    vis_marker_pub1 = nh.advertise<visualization_msgs::Marker>("plane_marker", 1, true);
    point_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("output_cloud", 10, true);
    point_cloud_pub1 = nh.advertise<sensor_msgs::PointCloud2>("output_cloud1", 10, true);
    point_cloud_pub2 = nh.advertise<sensor_msgs::PointCloud2>("output_cloud2", 10, true);

    ros::spin();
}