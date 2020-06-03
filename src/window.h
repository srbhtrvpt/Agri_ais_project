#ifndef _WINDOW_H_
#define _WINDOW_H_

#include <pcl_ros/point_cloud.h>

typedef pcl::PointXYZI PointT;

class Window
{
public:
    Window(float offset_x, float offset_y, float size_x, float size_y);
    bool is_inside(const PointT& p) const;
    
protected:
    float min_x, min_y, max_x, max_y;
    
};

#include "window.cpp"
#endif // _WINDOW_H_
