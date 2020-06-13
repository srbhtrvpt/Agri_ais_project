#include "window.h"

Window::Window(float size_x, float size_y, float offset_x, float offset_y) :
    min_x_(offset_x)
    , max_x_(offset_x + size_x)
    , min_y_(offset_y - 0.5*size_y)
    , max_y_(offset_y + 0.5*size_y)
{
}

bool Window::is_inside(float x, float y) const
{
    return (x >= min_x_ && x <= max_x_) && (y >= min_y_ && y <= max_y_);
}
