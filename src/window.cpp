#include "window.h"

Window::Window(float offset_x_, float offset_y_, float size_x_, float size_y_)
{
    min_x_ = offset_x_;
    max_x_ = offset_x_ + size_x_;
    min_y_ = offset_y_ - 0.5*size_y_;
    max_y_ = offset_y_ + 0.5*size_y_;
}

bool Window::is_inside(const PointT& p) const
{
    return (p.x >= min_x_ && p.x <= max_x_) && (p.y >= min_y_ && p.y <= max_y_);
}
