#include "tb_fbe_interface.hpp"

#include <hector_nav_msgs/GetRobotTrajectory.h>

namespace tb_interfaces {

FBEInterface::FBEInterface(ros::NodeHandle nh, ros::NodeHandle nh_private)
    : BaseInterface{nh, nh_private},
      ready{false}
{
    get_exploration_path = nh.serviceClient<hector_nav_msgs::GetRobotTrajectory>(EXPLORATION_PATH_SRV);
}

bool FBEInterface::plan_cb(tb_simulation::PlanPath::Request& req,
                           tb_simulation::PlanPath::Response& res) {
    target = req.target;

    replan(res.path);
    ready = true;
    return true;
}

void FBEInterface::target_reached_cb(const tb_simulation::TargetReached::ConstPtr& msg) {
    if (!ready || msg->path_id != path_id || (path_length > 1 && msg->target == 0)) return;

    ROS_INFO("Reached target %u in path %u, while we're at path %u (length: %u)",
             msg->target, msg->path_id, path_id, path_length);
    if (!replan()) {
        ROS_WARN_THROTTLE(1, "Failed to replan after reaching target!");
    }
}

bool FBEInterface::replan() {
    std::vector<geometry_msgs::Pose> path;
    return replan(path);
}

bool FBEInterface::replan(std::vector<geometry_msgs::Pose>& path) {
    hector_nav_msgs::GetRobotTrajectory call;
    if (!get_exploration_path.call(call)) {
        ROS_WARN_THROTTLE(1, "Failed to get exploration path!");
        return false;
    }

    path.clear();
    for (auto& p : call.response.trajectory.poses) {
        p.pose.position.z = 1.3;
        path.push_back(p.pose);
    }
    publish_path(path);
    path_length = path.size();

    return true;
}

} // namespace tb_interfaces

int main(int argc, char** argv) {
    ros::init(argc, argv, "fbePlanner");

    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    tb_interfaces::FBEInterface fbe(nh, nh_private);

    ros::spin();

    return 0;
}
