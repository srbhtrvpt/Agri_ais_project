#ifndef _WINDOW_H_
#define _WINDOW_H_

#include <pcl_ros/point_cloud.h>

typedef pcl::PointXYZI PointT;

class Window
{
public:
    Window();
    Window(float offset_x_, float offset_y_, float size_x_, float size_y_);
    bool is_inside(const PointT& p) const;
    
protected:
    float min_x_, min_y_, max_x_, max_y_;
    
};

#endif // _WINDOW_H_
