#include "tb_2d_interface.hpp"

#include <algorithm>
#include <tf/transform_datatypes.h>

namespace tb_interfaces {

Interface2D::Interface2D(ros::NodeHandle nh, ros::NodeHandle nh_private)
    : BaseInterface(nh, nh_private) {
    map_sub = nh.subscribe(MAP_TOPIC, 10, &Interface2D::map_cb, this);
}

void Interface2D::map_cb(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    if (header_changed(msg)) {
        map = *msg;
        ROS_INFO_THROTTLE(1, "Map header changed!");
        process_map_replaced();
        return;
    }

    std::vector<std::tuple<int, int, int8_t>> updated_cells;

    int width = map.info.width;
    int height = map.info.height;
    for (int x = 0; x < width; x++) {
        for (int y = 0; y < height; y++) {
            int8_t old_value = map.data[x + y * width];
            int8_t new_value = msg->data[x + y * width];
            if (old_value != new_value) {
                map.data[x + y * width] = new_value;
                updated_cells.push_back(std::make_tuple(x, y, new_value));
            }
        }
    }
    process_map_updates(updated_cells);

    ROS_INFO_THROTTLE(1, "%zu cells changed", updated_cells.size());
}

bool Interface2D::header_changed(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    const auto& ni = msg->info;
    const auto& oi = map.info;
    tf::Pose no;
    tf::Pose oo;
    tf::poseMsgToTF(ni.origin, no);
    tf::poseMsgToTF(oi.origin, oo);

    double ny = tf::getYaw(no.getRotation());
    double oy = tf::getYaw(oo.getRotation());
    ROS_ASSERT(ny == 0 || std::isnan(ny));

    return (
        ni.resolution != oi.resolution || ni.width != oi.width ||
        ni.height != oi.height || no.getOrigin() != oo.getOrigin() ||
        (!std::isnan(ny) && !std::isnan(oy) && ny != oy) ||
        (std::isnan(ny) && !std::isnan(oy)) || (!std::isnan(ny) && std::isnan(oy))
    );
}

std::tuple<double, double, double> Interface2D::calc_2d_coords(const geometry_msgs::Pose& g_pose) {
    tf::Pose pose;
    tf::poseMsgToTF(g_pose, pose);

    double x = pose.getOrigin().getX() - map.info.origin.position.x;
    x = std::min(map.info.origin.position.x + map.info.width, std::max(0.0, x));
    double y = pose.getOrigin().getY() - map.info.origin.position.y;
    y = std::min(map.info.origin.position.y + map.info.height, std::max(0.0, y));
    double theta = tf::getYaw(pose.getRotation());
    if (std::isnan(theta)) {
        theta = 0;
    }

    ROS_INFO_THROTTLE(1, "Pose: %f %f %f,\t2D Coords: %f %f %f",
                      pose.getOrigin().getX(), pose.getOrigin().getY(), theta,
                      x, y, theta);

    return std::make_tuple(x, y, theta);
}

geometry_msgs::Pose Interface2D::calc_pose(const std::tuple<double, double, double>& coords_2d) {
    const auto& mp = map.info.origin.position;
    tf::Pose tpose = tf::Pose(tf::createQuaternionFromYaw(std::get<2>(coords_2d)),
                              tf::Vector3(std::get<0>(coords_2d) + mp.x,
                                          std::get<1>(coords_2d) + mp.y, 1.3));
    geometry_msgs::Pose gpose;
    tf::poseTFToMsg(tpose, gpose);
    return gpose;
}

} // namespace tb_interfaces
