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
    // TODO: Use sbpl_xy_theta_pt_t?
    geometry_msgs::Pose calc_pose(const std::tuple<double, double, double>& coords_2d);

protected:
    virtual void map_cb(const nav_msgs::OccupancyGrid::ConstPtr& msg);

    virtual void process_map_updates(const std::vector<std::tuple<int, int, int8_t>>& updated_cells) = 0;
    virtual void process_map_replaced() = 0;

    ros::Subscriber map_sub;

    nav_msgs::OccupancyGrid map;

    bool header_changed(const nav_msgs::OccupancyGrid::ConstPtr& msg);
};

}

#endif // __TB_2D_INTERFACE_H_
