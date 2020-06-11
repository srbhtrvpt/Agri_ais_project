
template <class PointT>
Window<PointT>::Window( float size_x, float size_y, float offset_x, float offset_y)
{
    min_x_ = offset_x;
    max_x_ = offset_x + size_x;
    min_y_ = offset_y - 0.5*size_y;
    max_y_ = offset_y + 0.5*size_y;
}

template <class PointT>
bool Window<PointT>::is_inside(const PointT& p) const
{
    return (p.x >= min_x_ && p.x <= max_x_) && (p.y >= min_y_ && p.y <= max_y_);
}
