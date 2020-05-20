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

ros::Publisher point_cloud_pub;
ros::Publisher point_cloud_out_pub;
ros::Publisher point_cloud_curv_pub;
ros::Publisher vis_pub;

typedef pcl::PointXYZ PointT;
typedef pcl::Normal PointNT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointCloud<PointNT> PointCloudN;

PointCloud::Ptr cloud_in(new PointCloud), cloud_inliers(new PointCloud), cloud_outliers(new PointCloud);
sensor_msgs::PointCloud2::Ptr output_ground(new sensor_msgs::PointCloud2), output_plants(new sensor_msgs::PointCloud2), cloud_out(new sensor_msgs::PointCloud2);

pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
const int radius = 2; // neigh_radius
float nx, ny, nz, curvature;
int cc_prime = 0;
PointCloudN::Ptr cloud_normals(new PointCloudN);
PointCloud::iterator it;

std_msgs::Header header_cloud_out;
std::vector<sensor_msgs::PointField> fields;
sensor_msgs::PointField pt_field;

//sensor_msgs::PointField(name='median_curvature', offset=16, datatype=sensor_msgs::PointField::FLOAT32, count=1)}

//sensor_msgs::PointCloud2Modifier modifier(*cloud_out);

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
    for (int i = -radius; i <= radius; i++)
    {
        for (int j = -radius; j <= radius; j++)
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
    for (int i = -radius; i <= radius; i++)
    {
        if (point_valid(cc, rr + i))
        {
            result.push_back((rr + i) * cloud_in->width + (cc));
        }
    }
    for (int j = -radius; j <= radius; j++)
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
    /*uint8_t bytes[4];
    bytes[0] = (argn >> 24) & 0xFF;
    bytes[1] = (argn >> 16) & 0xFF;
    bytes[2] = (argn >> 8) & 0xFF;
    bytes[3] = argn & 0xFF;
    printf("0x%x 0x%x 0x%x 0x%x\n", (unsigned char)bytes[0],
    (unsigned char)bytes[1],
    (unsigned char)bytes[2],
    (unsigned char)bytes[3]); */


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
//void init_point_fields()

void callback(const sensor_msgs::PointCloud2ConstPtr &cloud)
{
    pcl::NormalEstimation<PointT, PointNT> ne;
    pcl::fromROSMsg(*cloud, *cloud_in);
    std::vector<int> map;
    cloud_in->is_dense = false;
    pcl::removeNaNFromPointCloud(*cloud_in, *cloud_in, map);
    int j = 0, counter = 0;

    visualization_msgs::Marker marker;
    marker.header.frame_id = cloud->header.frame_id;
    marker.header.stamp = cloud->header.stamp;
    marker.ns = "line_markers";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.color.a = 1.0;
    marker.scale.x = 0.01;
    marker.pose.orientation.w = 1.0;

    // std::cout << cloud_in->header.frame_id;
    // ros::Time now(cloud_in->header.stamp / 1e6, fmod(cloud_in->header.stamp, 1e6));
    // std::cout << cloud_in->points.size() << "......" << cloud_in->width << "......" << cloud_in->height  << "......" <<
    // cloud_in->width*cloud_in->height << "\n" ;

    if (cloud_in->isOrganized())
    {
        float curv_max = 0, curv_min = 1, curv_avg = 0;

        for (int rr = 0; rr < cloud_in->height; rr++)
        {
            for (int cc = 0; cc < cloud_in->width; cc++)
            {
                geometry_msgs::Point p;
                std_msgs::ColorRGBA color;
                /* marker.pose.position.x =  cloud_in->at(cc,rr).x;
                marker.pose.position.y = cloud_in->at(cc,rr).y;
                marker.pose.position.z = cloud_in->at(cc,rr).z;
                marker.pose.orientation.x = nx;
                marker.pose.orientation.y = ny;
                marker.pose.orientation.z = nz;
                marker.color.r = 0.0;
                marker.color.g = 1.0;
                marker.color.b = 0.0;*/

                // std::vector<int> indices = neighborhood_plus(cc, rr);
                std::vector<int> indices = neighborhood_sq(cc, rr);
                // std::cout << indices.size() << "...." << cc << "...." << rr << "\n";
                ne.computePointNormal(*cloud_in, indices, nx, ny, nz, curvature);
                PointNT normal_out;
                normal_out.normal_x = nx;
                normal_out.normal_y = ny;
                normal_out.normal_z = nz;
                normal_out.curvature = curvature;
                cloud_normals->push_back(normal_out);
                if (isfinite(curvature))
                {
                    if (curv_min > curvature)
                    {
                        curv_min = curvature;
                    }
                    if (curv_max < curvature)
                    {
                        curv_max = curvature;
                    }
                    curv_avg += curvature;
                }
                // std::cout << "x" << nx << "y" << ny << "z" << nz << "......" << indices.size() << "curv" << curvature << "cc"
                // << cc << "rr" << rr << "\n";

                if (point_valid(cc, rr) && isfinite(nz) && isfinite(ny) && isfinite(nx) && isfinite(curvature))
                {
                    p.x = cloud_in->at(cc, rr).x;
                    p.y = cloud_in->at(cc, rr).y;
                    p.z = cloud_in->at(cc, rr).z;

                    if (curvature < 0.00275)
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

                    marker.points.push_back(p);
                    marker.colors.push_back(color);
                    p.z += nz;
                    p.x += nx;
                    p.y += ny;
                    // p.z += 1;
                    marker.points.push_back(p);
                    marker.colors.push_back(color);
                }
            }
        }

        std::cout << "curv_avg" << curv_avg / cloud_in->points.size() << "curv_min" << curv_min << "curv_max" << curv_max
                  << "\n";
    }
    else
    {
        /* std::vector<int> indices;
    cloud_in->is_dense = false;
    pcl::removeNaNFromPointCloud(*cloud_in, *cloud_in, indices); */

        ne.setInputCloud(cloud_in);
        pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
        ne.setSearchMethod(tree);
        // ne.setRadiusSearch(0.3);
        ne.setKSearch(25);
        ne.compute(*cloud_normals);
        /*std::vector<int> map1;
        cloud_normals->is_dense = false;
        pcl::removeNaNFromPointCloud(*cloud_normals, *cloud_normals, map1); */

        /*
        modifier.resize(cloud_normals->points.size()); 
        modifier.setPointCloud2FieldsByString(2, "xyz", 
                                            "curvature");
        sensor_msgs::PointCloud2Iterator<double> iter_x(*cloud_out, "x");
        sensor_msgs::PointCloud2Iterator<double> iter_y(*cloud_out, "y");
        sensor_msgs::PointCloud2Iterator<double> iter_z(*cloud_out, "z");
        sensor_msgs::PointCloud2Iterator<double> iter_c(*cloud_out, "curvature"); */
        /*marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;*/
        // pcl::toROSMsg(*cloud_normals, *output_ground);

        header_cloud_out.frame_id = cloud_in->header.frame_id;
        header_cloud_out.stamp = cloud->header.stamp;

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
        pt_field.name = 'c';
        pt_field.offset = 12;
        pt_field.datatype = pt_field.FLOAT32;
        pt_field.count = 1;
        fields.push_back(pt_field);

        //uint8_t *point_binary;
        std::vector<uint8_t> point_binary;
        std::vector<uint8_t> points;

        for (int i = 0; i < cloud_normals->points.size(); i++)
        {
            std_msgs::ColorRGBA color;
            geometry_msgs::Point p;

            if (isfinite(cloud_in->points[i].x) && isfinite(cloud_in->points[i].y) && isfinite(cloud_in->points[i].z) &&
                isfinite(cloud_normals->points[i].normal_z) && isfinite(cloud_normals->points[i].normal_y) &&
                isfinite(cloud_normals->points[i].normal_x) && cloud_normals->points[i].curvature)
            {   
                counter++;
                if (cloud_normals->points[i].curvature < 0.00275)
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
                marker.points.push_back(p);
                marker.colors.push_back(color);
                p.z += cloud_normals->points[i].normal_z;
                p.y += cloud_normals->points[i].normal_y;
                p.x += cloud_normals->points[i].normal_x;
                marker.points.push_back(p);
                marker.colors.push_back(color);
                //std::cout << "x" <<  cloud_in->points[i].x << "x" <<  cloud_in->points[i].y << "x" <<  cloud_in->points[i].z << "i" << i << std::endl;

                /*
                ++iter_x;
                ++iter_y;
                ++iter_z;
                ++iter_c;
                *iter_x = cloud_in->points[i].x;
                *iter_y = cloud_in->points[i].x;
                *iter_z = cloud_in->points[i].x;
                *iter_c = cloud_normals->points[i].curvature; */

                //point_binary = reinterpret_cast<uint8_t *>(&cloud_in->points[i].x)

                point_binary = floattoeight(cloud_in->points[i].x);
                for (j = 0; j < 4; j++)
                {
                    points.push_back(point_binary[j]);
                }
                point_binary = floattoeight(cloud_in->points[i].y);
                for (j = 0; j < 4; j++)
                {
                    points.push_back(point_binary[j]);
                }
                point_binary = floattoeight(cloud_in->points[i].z);
                for (j = 0; j < 4; j++)
                {
                    points.push_back(point_binary[j]);
                }
                point_binary = floattoeight(cloud_normals->points[i].curvature);
                for (j = 0; j < 4; j++)
                {
                    points.push_back(point_binary[j]);
                }
            }
        }
        //std::cout << "count" <<  counter << "size" << cloud_normals->points.size() <<  "points" << points.size() <<  "\n";

        cloud_out->header = header_cloud_out;
        cloud_out->fields = fields;
        cloud_out->is_bigendian = false;
        cloud_out->width = counter;
        cloud_out->height = 1;
        cloud_out->data = points;
        cloud_out->point_step = 16;
        cloud_out->row_step = counter;
    }

    point_cloud_curv_pub.publish(cloud_out);
    vis_pub.publish(marker);

    // plane model
    /*coefficients->values.resize(4);
  pcl::SACSegmentation<PointT> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setDistanceThreshold(0.05);*/

    

    //extract
    pcl::ExtractIndices<PointT> extract;



    // plane normal model
    pcl::SACSegmentationFromNormals<PointT, PointNT> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);
    seg.setNormalDistanceWeight(0.1);
    seg.setDistanceThreshold(0.05);
    seg.setInputNormals(cloud_normals);

    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setInputCloud(cloud_in);
    seg.segment(*inliers, *coefficients);

    // std::cout << "size"<<inliers->indices.size();

    if (inliers->indices.size() == 0)
    {
        PCL_ERROR("Could not estimate a planar model for the given dataset.\n");
    }
    // Extract inliers
    else
    {
        extract.setInputCloud(cloud_in);
        extract.setIndices(inliers);
        extract.setNegative(false); // Extract the inliers
        extract.filter(*cloud_inliers);
        pcl::toROSMsg(*cloud_inliers, *output_ground); // cloud_inliers contains the plane
        point_cloud_out_pub.publish(output_ground);
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
    // std::cout << PCL_VERSION << std::endl;

    // ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/sensor/laser/vlp16/front/pointcloud_xyzi", 1,
    // callback);

    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/point_cloud_decay", 10, callback);
    vis_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1, true);
    point_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("point_cloud_ground", 10, true);
    point_cloud_out_pub = nh.advertise<sensor_msgs::PointCloud2>("point_cloud_plants", 10, true);
    point_cloud_curv_pub = nh.advertise<sensor_msgs::PointCloud2>("point_cloud_curv", 10, true);


    ros::spin();
}