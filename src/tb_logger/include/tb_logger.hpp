#ifndef __TB_LOGGER_H_
#define __TB_LOGGER_H_

#include <fstream>
#include <filesystem>
namespace fs = std::filesystem;

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/Duration.h>
#include <tb_logger/Notify.h>

namespace tb_logger {

constexpr const char* POSE_TOPIC = "pose";
constexpr const char* MAP_TOPIC = "/projected_map";
constexpr const char* PLANNING_TOPIC = "planning_time";
constexpr const char* NOTIFY_SRV = "notify";

#pragma pack(1)
struct point {
    double x;
    double y;
    double z;
};

struct bindata_1 {
    point p;
    double o_x;
    double o_y;
    double o_z;
    double o_w;
};

struct bindata_30_header {
    double planning_time;
    point map_origin;
    uint32_t map_width;
    uint32_t map_height;
};
#pragma options align=reset

class Logger {
public:
    Logger(ros::NodeHandle nh, ros::NodeHandle nh_private);

    fs::path get_data_dir(bool mkdir = true);

protected:
    void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void map_cb(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void planning_cb(const std_msgs::Duration::ConstPtr& msg);
    bool notify_cb(Notify::Request& req, Notify::Response&);
    void timer_1_s_cb(const ros::TimerEvent&);
    void timer_30_s_cb(const ros::TimerEvent&);

    bool started(const std::string& msg);
    bool finished(const std::string& msg);
    bool aborted(const std::string& msg);

    void write_1_s_data();
    void write_30_s_data();

    ros::NodeHandle nh;
    ros::NodeHandle nh_private;

    ros::Subscriber pose_sub;
    ros::Subscriber map_sub;
    ros::Subscriber planning_sub;
    ros::ServiceServer notify_srv;
    ros::Timer data_1_s;
    ros::Timer data_30_s;

    geometry_msgs::Pose pose;
    nav_msgs::OccupancyGrid map;
    ros::Duration planning_time_acc;

    bool ready;
    std::string uuid;
    ros::Time start;

    std::fstream bin_1_s;
    std::fstream bin_30_s;
    std::fstream csv;
};

} // namespace tb_logger

#endif // __TB_LOGGER_H_
