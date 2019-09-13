#include "tb_logger.hpp"

#include <boost/uuid/uuid_io.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <ros/package.h>

namespace tb_logger {

Logger::Logger(ros::NodeHandle nh, ros::NodeHandle nh_private)
    : nh{nh},
      nh_private{nh_private},
      ready{false},
      uuid{to_string(boost::uuids::random_generator()())}
{
    ROS_INFO("Starting simulation %s", uuid.c_str());

    pose_sub = nh_private.subscribe(POSE_TOPIC, 10, &Logger::pose_cb, this);
    map_sub = nh_private.subscribe(MAP_TOPIC, 10, &Logger::map_cb, this);
    planning_sub = nh_private.subscribe(PLANNING_TOPIC, 10, &Logger::planning_cb, this);
    notify_srv = nh_private.advertiseService(NOTIFY_SRV, &Logger::notify_cb, this);

    fs::path data_dir = get_data_dir();
    std::string bin_prefix = data_dir / uuid;
    std::string csv_path = data_dir / "list.csv";
    bin_1_s = std::fstream(bin_prefix + "1_s.bin", std::ios::out | std::ios::binary | std::ios::trunc);
    bin_30_s = std::fstream(bin_prefix + "30_s.bin", std::ios::out | std::ios::binary | std::ios::trunc);
    csv = std::fstream(csv_path, std::ios::out | std::ios::app);

    // TODO: Add timers for writing out data
}

fs::path Logger::get_data_dir(bool mkdir) {
    fs::path path = fs::path(ros::package::getPath("tb_logger")) / "data";

    if (mkdir) {
        fs::create_directories(path);
    }

    return path;
}

void Logger::pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    pose = msg->pose;
}

void Logger::map_cb(const nav_msgs::OccupancyGrid::ConstPtr &msg) {
    map = *msg;
}

void Logger::planning_cb(const std_msgs::Duration::ConstPtr &msg) {
    planning_time_acc += msg->data;
}

bool Logger::notify_cb(Notify::Request& req, Notify::Response&) {
    switch (req.state) {
    case Notify::Request::STARTED:
        return started(req.msg);
    case Notify::Request::FINISHED:
        return finished(req.msg);
    case Notify::Request::ABORTED:
        return aborted(req.msg);
    default:
        ROS_ERROR("Unknown notify state %u: %s", req.state, req.msg.c_str());
        return false;
    }
}

bool Logger::started(const std::string& msg) {
    if (ready) {
        ROS_WARN("Got STARTED message, but already started?! (%s)", msg.c_str());
    } else {
        ROS_INFO("Starting!");
        start = ros::Time::now();
        ready = true;
    }

    return true;
}

bool Logger::finished(const std::string& msg) {
    if (ready) {
        // TODO: Write out last data points & CSV entry
        ROS_INFO("Finished: %s", msg.c_str());
        ready = false;
    } else {
        ROS_WARN("Got FINISHED message, but never started?! (%s)", msg.c_str());
    }

    return true;
}

bool Logger::aborted(const std::string& msg) {
    if (ready) {
        // TODO: Write out last data points & CSV entry
        ROS_INFO("Aborted: %s", msg.c_str());
        ready = false;
    } else {
        ROS_WARN("Got ABORTED message, but never started?! (%s)", msg.c_str());
    }

    return true;
}

} // namespace tb_logger

int main(int argc, char** argv) {
    ros::init(argc, argv, "tbLogger");

    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    tb_logger::Logger logger(nh, nh_private);

    ros::spin();

    return 0;
}
