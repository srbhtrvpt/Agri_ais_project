#ifndef _WINDOW_H_
#define _WINDOW_H_

class Window
{
public:
    Window(float offset_x, float offset_y, float size_x, float size_y);
    bool is_inside(const PointT& p) const;
    
protected:
    float min_x_, min_y_, max_x_, max_y_;
    
};


#endif // _WINDOW_H_
