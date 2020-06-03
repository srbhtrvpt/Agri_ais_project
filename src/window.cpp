#include "window.h"

Window::Window(float offset_x, float offset_y, float size_x, float size_y)
{

    float min_x = offset_x;
    float max_x = offset_x + size_x;
    float min_y = offset_y - 0.5*size_y;
    float max_y = offset_y + 0.5*size_y;
}

bool Window::is_inside(const PointT& p) const
{
    
    return (p.x >= min_x && p.x <= max_x) && (p.y >= min_y && p.y <= max_y);

}
