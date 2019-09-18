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
    bin_1_s = std::fstream(bin_prefix + "_1.bin", std::ios::out | std::ios::binary | std::ios::trunc);
    bin_30_s = std::fstream(bin_prefix + "_30.bin", std::ios::out | std::ios::binary | std::ios::trunc);
    final_map = std::fstream(bin_prefix + "_map.bin", std::ios::out | std::ios::binary | std::ios::trunc);
    csv = std::fstream(csv_path, std::ios::out | std::ios::app);
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

    // Rebuild BBX indices
    indoor_area = 0;
    for (int i = 0; i < 4; i++) {
        indoor_bbxs[i][0] = (INDOOR_BBXS_D[i][0] - map.info.origin.position.x) /
                            map.info.resolution;
        indoor_bbxs[i][1] = (INDOOR_BBXS_D[i][1] - map.info.origin.position.y) /
                            map.info.resolution;
        indoor_bbxs[i][2] = (INDOOR_BBXS_D[i][2] - map.info.origin.position.x) /
                            map.info.resolution;
        indoor_bbxs[i][3] = (INDOOR_BBXS_D[i][3] - map.info.origin.position.y) /
                            map.info.resolution;
        indoor_area += (indoor_bbxs[i][2] - indoor_bbxs[i][0]) *
                       (indoor_bbxs[i][3] - indoor_bbxs[i][1]);
    }
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

void Logger::timer_1_s_cb(const ros::TimerEvent&) {
    if (ready) {
        write_1_s_data();
    }
}

void Logger::timer_30_s_cb(const ros::TimerEvent&) {
    if (ready) {
        write_30_s_data();
    }
}

bool Logger::started(const std::string& msg) {
    if (ready) {
        ROS_WARN("Got STARTED message, but already started?! (%s)", msg.c_str());
    } else {
        ROS_INFO("Starting!");
        start = ros::Time::now();
        start_msg = msg;
        ready = true;
        data_1_s = nh_private.createTimer(ros::Duration(1),
                                          &Logger::timer_1_s_cb, this);
        data_30_s = nh_private.createTimer(ros::Duration(30),
                                           &Logger::timer_30_s_cb, this);
    }

    return true;
}

bool Logger::finished(const std::string& msg) {
    if (ready) {
        write_end_data(false, msg);
        ROS_INFO("Finished: %s", msg.c_str());
        ready = false;
    } else {
        ROS_WARN("Got FINISHED message, but never started?! (%s)", msg.c_str());
    }

    return true;
}

bool Logger::aborted(const std::string& msg) {
    if (ready) {
        write_end_data(true, msg);
        ROS_INFO("Aborted: %s", msg.c_str());
        ready = false;
    } else {
        ROS_WARN("Got ABORTED message, but never started?! (%s)", msg.c_str());
    }

    return true;
}

void Logger::write_1_s_data() {
    auto er = map_exploration_ratio();
    bindata_1 data = {
        {pose.position.x, pose.position.y, pose.position.z},
        pose.orientation.x, pose.orientation.y, pose.orientation.z,
        pose.orientation.w,
        er.first, er.second
    };
    bin_1_s.write((char*) &data, sizeof(data));
    bin_1_s.sync();
}

void Logger::write_30_s_data() {
    double planning_time = planning_time_acc.sec +  \
                           ((double) planning_time_acc.nsec) / 1E9;
    planning_time_acc.sec = planning_time_acc.nsec = 0;

    bindata_30_header header = {
        planning_time,
        {map.info.origin.position.x, map.info.origin.position.y,
         map.info.origin.position.z},
        map.info.width, map.info.height
    };
    bin_30_s.write((char*) &header, sizeof(header));
    bin_30_s.write((char*) map.data.data(), map.info.width * map.info.height);
    bin_30_s.sync();
}

void Logger::write_end_data(bool aborted, const std::string& reason) {
    final_map.write((char*) &map.header, sizeof(map.header));
    final_map.write((char*) &map.info, sizeof(map.info));
    final_map.write((char*) &map.data, map.info.width * map.info.height);
    final_map.sync();

    csv << uuid << ";" << start_msg << ";" << (ros::Time::now() - start).toSec()
        << ";" << aborted << ";" << reason << ";" << planning_time_acc.toSec()
        << "\n";
    csv.sync();
}

std::pair<double, double> Logger::map_exploration_ratio() {
    uint64_t explored = 0;
    uint64_t explored_indoor = 0;

    for (uint32_t y = 0; y < map.info.height; y++) {
        for (uint32_t x = 0; x < map.info.width; x++) {
            if (map.data[y * map.info.width + x] != -1) {
                explored++;
                for (int i = 0; i < 4; i++) {
                    if (x > indoor_bbxs[i][0] && x < indoor_bbxs[i][2] &&
                        y > indoor_bbxs[i][1] && y < indoor_bbxs[i][3]) {
                        explored_indoor++;
                        break;
                    }
                }
            }
        }
    }

    return std::make_pair(explored / (double) (map.info.width * map.info.height),
                          explored_indoor / (double) indoor_area);
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
