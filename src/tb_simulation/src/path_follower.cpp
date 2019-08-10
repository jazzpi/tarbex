#include "path_follower.hpp"

#include <tb_simulation/PlanPath.h>

namespace tb_simulation {

#define ROS_INFO_POSE_THROTTLE(rate, name, pose) {  \
        auto orig = pose.getOrigin();                                   \
        auto rot = pose.getRotation();                                  \
        ROS_INFO_THROTTLE(rate, "%s: [%f, %f, %f], rot: (%f, %f, %f, %f)", \
                          name, orig.x(), orig.y(), orig.z(), rot.x(), rot.y(), rot.z(), rot.w()); \
    }

PathFollower::PathFollower(ros::NodeHandle nh, ros::NodeHandle nh_private)
    : nh{nh},
      nh_private{nh_private},
      ready{false},
      plan_seq{0}
{
    current_target = path.end();

    start = nh.advertiseService(START_SRV, &PathFollower::start_callback, this);
    planner = nh.serviceClient<PlanPath>(PLANNER_SRV);
    planner.waitForExistence();

    wp_pub = nh.advertise<geometry_msgs::Pose>(WAYPOINT_TOPIC, 1);
    pose_sub = nh.subscribe(POSE_TOPIC, 10, &PathFollower::pose_callback, this);
}

bool PathFollower::start_callback(std_srvs::Trigger::Request&,
                                  std_srvs::Trigger::Response& res) {
    if (ready) {
        res.success = false;
        res.message = "Already started";
    } else {
        ready = true;
        res.success = true;
        res.message = "Started";
    }

    return true;
}

void PathFollower::pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    if (!ready) {
        return;
    }

    if (current_target == path.end()) {
        ROS_INFO("No target... Planning.");
        plan();
    }

    tf::Pose pose;
    tf::poseMsgToTF(msg->pose, pose);
    tf::Pose diff = current_target->inverseTimes(pose);

    if (diff.getOrigin().length() < PATH_ORIG_EPSILON &&
        tf::getYaw(diff.getRotation()) < PATH_YAW_EPSILON) {
        ROS_INFO_THROTTLE(1, "Reached target!");
        ROS_INFO_POSE_THROTTLE(1, "target", (*current_target));
        ROS_INFO_POSE_THROTTLE(1, "pose", pose);
        advance_target();
    }
    ROS_INFO_POSE_THROTTLE(1, "diff", diff);
}

void PathFollower::plan() {
    tb_simulation::PlanPath srv;
    srv.request.header.seq = plan_seq++;
    srv.request.header.stamp = ros::Time::now();
    srv.request.header.frame_id = FRAME_ID;

    if (planner.call(srv)) {
        path.clear();
        for (const auto& pose : srv.response.path) {
            tf::Pose tp;
            tf::poseMsgToTF(pose, tp);
            path.push_back(tp);
        }
        current_target = path.begin();
        publish_pose(*current_target);
    } else {
        ROS_WARN_THROTTLE(1, "Couldn't reach the planner service at %s", PLANNER_SRV);
    }
}

void PathFollower::advance_target() {
    current_target++;
    if (current_target == path.end()) {
        plan();
    } else {
        publish_pose(*current_target);
    }
}

void PathFollower::publish_pose(const tf::Pose& tp) {
    geometry_msgs::Pose pose;
    tf::poseTFToMsg(tp, pose);
    wp_pub.publish(pose);
}

} // namespace tb_simulation

int main(int argc, char** argv) {
    ros::init(argc, argv, "pathFollower");

    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    tb_simulation::PathFollower planner(nh, nh_private);

    ros::spin();
    return 0;
}
