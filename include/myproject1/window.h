#ifndef _WINDOW_H_
#define _WINDOW_H_

#include <string>

class Window
{
public:
    Window(std::string frame_id, float size_x = 1.f, float size_y = 1.f, float offset_x= 0.f, float offset_y = 0.f);
    bool is_inside(float x, float y) const;
    std::string frame_id() const;
    
protected:
    float min_x_, min_y_, max_x_, max_y_;
    std::string frame_id_;
    
};

#endif // _WINDOW_H_
