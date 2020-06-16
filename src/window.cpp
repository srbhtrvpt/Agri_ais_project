#include "window.h"

#include <iostream>

Window::Window(std::string frame_id, float size_x, float size_y, float offset_x, float offset_y) :
    frame_id_(frame_id)
    ,min_x_(offset_x)
    , max_x_(offset_x + size_x)
    , min_y_(offset_y - 0.5*size_y)
    , max_y_(offset_y + 0.5*size_y)
{
    std::cout << "Window dimensions: [" << min_x_ << "," << max_x_ << "]x[" << min_y_ << "," << max_y_ << "]" << std::endl;
}

bool Window::is_inside(float x, float y) const
{
    return (x >= min_x_ && x <= max_x_) && (y >= min_y_ && y <= max_y_);
}

std::string Window::frame_id() const
{
    return frame_id_;
}
