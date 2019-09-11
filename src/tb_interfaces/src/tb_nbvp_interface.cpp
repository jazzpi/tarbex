#include "tb_nbvp_interface.hpp"

#include <nbvplanner/nbvp_srv.h>

namespace tb_interfaces {

#define ROS_INFO_POSE_THROTTLE(rate, name, pose) {  \
        auto orig = pose.getOrigin();                                   \
        auto rot = pose.getRotation();                                  \
        ROS_INFO_THROTTLE(rate, "%s: [%f, %f, %f], rot: (%f, %f, %f, %f)", \
                          name, orig.x(), orig.y(), orig.z(), rot.x(), rot.y(), rot.z(), rot.w()); \
    }

NBVPInterface::NBVPInterface(ros::NodeHandle nh, ros::NodeHandle nh_private)
    : BaseInterface{nh, nh_private},
      ready{false}
{
    get_exploration_path = nh.serviceClient<nbvplanner::nbvp_srv>(NBVP_SRV);
    get_exploration_path.waitForExistence();
}

bool NBVPInterface::plan_cb(tb_simulation::PlanPath::Request& req,
                            tb_simulation::PlanPath::Response& res) {
    target = req.target;
    publish_target_vis(target);

    replan(res.path);
    ready = true;

    return true;
}

void NBVPInterface::target_reached_cb(const tb_simulation::TargetReached::ConstPtr& msg) {
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
    nbvplanner::nbvp_srv call;
    call.request.header.seq = plan_seq++;
    call.request.header.stamp = ros::Time::now();
    call.request.header.frame_id = FRAME_ID;

    if (!get_exploration_path.call(call)) {
        ROS_WARN_THROTTLE(1, "Couldn't reach the planner service at %s", NBVP_SRV);
        return false;
    } else if (call.response.path.size() == 0) {
        ROS_WARN_THROTTLE(1, "Planner service returned a path of size 0!");
        return false;
    } else {
        path = call.response.path;
        publish_path(path);
    }

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
