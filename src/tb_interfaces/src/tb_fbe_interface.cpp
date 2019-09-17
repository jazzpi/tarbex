#include "tb_fbe_interface.hpp"

#include <nav_msgs/GetPlan.h>

namespace tb_interfaces {

FBEInterface::FBEInterface(ros::NodeHandle nh, ros::NodeHandle nh_private)
    : BaseInterface{nh, nh_private},
      ready{false}
{
    get_exploration_path = nh.serviceClient<nav_msgs::GetPlan>(EXPLORATION_PATH_SRV);
}

bool FBEInterface::plan_cb(tb_simulation::PlanPath::Request& req,
                           tb_simulation::PlanPath::Response& res) {
    target = req.target;
    publish_target_vis(target);

    replan(res.path);
    ready = true;
    return true;
}

void FBEInterface::target_reached_cb(const tb_simulation::TargetReached::ConstPtr& msg) {
    if (!ready || msg->path_id != path_id || (path_length > 1 && msg->target == 0)) return;

    ROS_INFO("Reached target %zu in path %u, while we're at path %u (length: %zu)",
             msg->target, msg->path_id, path_id, path_length);
    if (!replan()) {
        ROS_WARN_THROTTLE(1, "Failed to replan after reaching target!");
        failed_replans = 0;
        replan_timer.start();
    }
}

void FBEInterface::replan_timer_cb(const ros::TimerEvent&) {
    if (!replan()) {
        failed_replans++;
        if (failed_replans >= REPLAN_TRIES_MAX) {
            ROS_FATAL("Failed to replan %u times. Aborting.", failed_replans);
            ros::shutdown();
            exit(1);
        }
        ROS_WARN("Failed to replan %u times", failed_replans);
    } else {
        ROS_INFO("Recovered after failing to replan %u times", failed_replans);
        replan_timer.stop();
        failed_replans = 0;
    }
}

bool FBEInterface::replan() {
    std::vector<geometry_msgs::Pose> path;
    return replan(path);
}

bool FBEInterface::replan(std::vector<geometry_msgs::Pose>& path) {
    planning_start();

    nav_msgs::GetPlan call;
    call.request.goal.header.frame_id = "/map";
    call.request.goal.pose = target;
    if (!get_exploration_path.call(call)) {
        ROS_WARN_THROTTLE(1, "Failed to get exploration path!");
        return false;
    }

    path.clear();
    auto& traj = call.response.plan.poses;
    if (traj.size() < 3) {
        for (auto& p : traj) {
            p.pose.position.z = 1.3;
            path.push_back(p.pose);
        }
    } else {
        /* When the direction doesn't change, only keep first pose */
        auto last_dir = normalized_dir(traj[0].pose, traj[1].pose);
        auto first_p = traj.front().pose;
        first_p.position.z = 1.3;
        path.push_back(first_p);
        std::vector<geometry_msgs::Pose> erased;
        for (size_t i = 2; i < traj.size(); i++) {
          auto p = traj[i - 1].pose;
          auto dir = normalized_dir(p, traj[i].pose);
          p.position.z = 1.3;
          if ((dir - last_dir).length() <= DIR_DIFF_EPS) {
              erased.push_back(p);
          } else {
              path.push_back(p);
          }
          last_dir = dir;
        }
        auto last_p = traj.back().pose;
        last_p.position.z = 1.3;
        path.push_back(last_p);
        publish_path_vis(erased, 1, true);
        ROS_INFO_THROTTLE(
            1, "Erased %zu points from the trajectory. Publishing %zu/%zu points",
            erased.size(), path.size(), traj.size()
        );
    }

    planning_done();
    publish_path(path);
    path_length = path.size();

    // TODO: Check if we've reached the target here?
    return path_length > 0;
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
