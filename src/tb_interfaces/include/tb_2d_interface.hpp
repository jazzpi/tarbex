#ifndef __TB_2D_INTERFACE_H_
#define __TB_2D_INTERFACE_H_

#include "tb_base_interface.hpp"

#include <nav_msgs/OccupancyGrid.h>

namespace tb_interfaces {

constexpr const char* MAP_TOPIC = "/projected_map";

class Interface2D : public BaseInterface {
public:
    Interface2D(ros::NodeHandle nh, ros::NodeHandle nh_private);

    std::tuple<double, double, double> calc_2d_coords(const geometry_msgs::Pose& pose);

protected:
    virtual void map_cb(const nav_msgs::OccupancyGrid::ConstPtr& msg);

    ros::Subscriber map_sub;

    nav_msgs::OccupancyGrid map;

    bool header_changed(const nav_msgs::OccupancyGrid::ConstPtr& msg);
};

}

#endif // __TB_2D_INTERFACE_H_
