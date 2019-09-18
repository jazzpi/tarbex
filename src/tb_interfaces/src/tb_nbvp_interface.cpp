#include "tb_nbvp_interface.hpp"

#include <tb_simulation/PlanPath.h>

namespace tb_interfaces {

#define ROS_INFO_POSE_THROTTLE(rate, name, pose) {  \
        auto orig = pose.getOrigin();                                   \
        auto rot = pose.getRotation();                                  \
        ROS_INFO_THROTTLE(rate, "%s: [%f, %f, %f], rot: (%f, %f, %f, %f)", \
                          name, orig.x(), orig.y(), orig.z(), rot.x(), rot.y(), rot.z(), rot.w()); \
    }

NBVPInterface::NBVPInterface(ros::NodeHandle nh, ros::NodeHandle nh_private)
    : BaseInterface{nh, nh_private},
      ready{false},
      planning{false},
      last_plan{0}
{
    get_exploration_path = nh.serviceClient<tb_simulation::PlanPath>(NBVP_SRV);
    get_exploration_path.waitForExistence();
}

bool NBVPInterface::plan_cb(tb_simulation::PlanPath::Request& req,
                            tb_simulation::PlanPath::Response& res) {
    target = req.target;
    publish_target_vis(target);

    ROS_INFO("Replanning because of service call");
    replan(res.path);
    ready = true;

    return true;
}

void NBVPInterface::target_reached_cb(const tb_simulation::TargetReached::ConstPtr& msg) {
    if (!ready) return;

    if (planning) {
        ROS_WARN_THROTTLE(1, "Reached target, but we're still planning?!");
        return;
    } else if (last_plan > msg->header.stamp) {
        ROS_WARN_THROTTLE(1, "Reached target before we finished planning");
        return;
    }

    ROS_INFO("Replanning because we reached the target");
    if (!replan()) {
        ROS_WARN_THROTTLE(1, "Failed to replan after reaching target!");
        // TODO: Start replan timer?
    }
}

bool NBVPInterface::replan() {
    std::vector<geometry_msgs::Pose> path;
    return replan(path);
}

bool NBVPInterface::replan(std::vector<geometry_msgs::Pose>& path) {
    planning = true;
    planning_start();

    tb_simulation::PlanPath call;
    call.request.header.seq = plan_seq++;
    call.request.header.stamp = ros::Time::now();
    call.request.header.frame_id = FRAME_ID;
    call.request.target = target;

    if (!get_exploration_path.call(call)) {
        // No planning -> No need to publish planning time
        ROS_WARN_THROTTLE(1, "Couldn't reach the planner service at %s", NBVP_SRV);
        planning = false;
        return false;
    } else if (call.response.path.size() == 0) {
        planning_done();
        ROS_WARN_THROTTLE(1, "Planner service returned a path of size 0!");
        planning = false;
        return false;
    } else {
        planning_done();
        path = call.response.path;
        publish_path(path);
        planning = false;
    }

    last_plan = ros::Time::now();
    return true;
}

} // namespace tb_simulation

int main(int argc, char** argv) {
    ros::init(argc, argv, "nbvpInterface");

    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    tb_interfaces::NBVPInterface nbvp(nh, nh_private);

    ros::spin();
    return 0;
}
