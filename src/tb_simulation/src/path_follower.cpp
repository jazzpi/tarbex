#include "path_follower.hpp"

#include <tb_simulation/PlanPath.h>
#include <tb_simulation/TargetReached.h>

namespace tb_simulation {

#define ROS_INFO_POSE_THROTTLE(rate, name, pose) {  \
        auto orig = (pose).getOrigin();                                 \
        auto rot = (pose).getRotation();                                \
        ROS_INFO_THROTTLE(rate, "%s: [%f, %f, %f], rot: (%f, %f, %f, %f)", \
                          name, orig.x(), orig.y(), orig.z(), rot.x(), rot.y(), rot.z(), rot.w()); \
    }

PathFollower::PathFollower(ros::NodeHandle nh, ros::NodeHandle nh_private)
    : nh{nh},
      nh_private{nh_private},
      ready{false},
      last_wp_pub{ros::Time(0)},
      path_id{(uint32_t) -1}
{
    init_params();
    current_target = path.end();

    wp_pub = nh.advertise<geometry_msgs::Pose>(WAYPOINT_TOPIC, 10);
    tgt_pub = nh.advertise<TargetReached>(TGT_REACHED_TOPIC, 1);
    pose_sub = nh_private.subscribe(POSE_TOPIC, 10, &PathFollower::pose_callback, this);
    path_sub = nh_private.subscribe(PATH_TOPIC, 10, &PathFollower::path_callback, this);
}

void PathFollower::init_params() {
    nh_private.param<double>("path_orig_epsilon", path_orig_epsilon, 0.1);
    nh_private.param<double>("path_yaw_epsilon", path_yaw_epsilon, 0.01);
    nh_private.param<double>("wp_throttle", wp_throttle, 1.0);
}

void PathFollower::pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    if (!ready) {
        return;
    }

    if (current_target == path.end()) {
        ROS_WARN_DELAYED_THROTTLE(5, "No target!");
        return;
    }

    tf::Pose pose;
    tf::poseMsgToTF(msg->pose, pose);
    tf::Pose diff = current_target->inverseTimes(pose);

    bool advanced = false;
    while (current_target != path.end() &&
           diff.getOrigin().length() < path_orig_epsilon &&
           std::fabs(tf::getYaw(diff.getRotation())) < path_yaw_epsilon) {
        ROS_INFO("Reached target with yaw diff: %f!", tf::getYaw(diff.getRotation()));
        advance_target(false);
        advanced = true;
        diff = current_target->inverseTimes(pose);
    }
    if (advanced) {
        publish_target_reached();
    }
    if (current_target != path.end()) {
        publish_pose(*current_target, true);
    }
}

void PathFollower::path_callback(const geometry_msgs::PoseArray::ConstPtr& msg) {
    tf::Transform old_tgt;
    bool old_tgt_exists = false;
    if (path.size() > 0) {
        old_tgt = *current_target;
        old_tgt_exists = true;
    }

    path.clear();
    for (const auto &pose : msg->poses) {
      tf::Pose tp;
      tf::poseMsgToTF(pose, tp);
      path.push_back(tp);
    }
    if (path.size() > 0) {
        if (!old_tgt_exists || !(old_tgt == path[0])) {
            publish_pose(path[0]);
        }
    } else {
        // TODO: Check if we reached the target
        ROS_WARN_THROTTLE(1, "Path is empty!");
    }
    current_target = path.begin();
    ready = true;
    path_id = msg->header.seq;
}

void PathFollower::advance_target(bool do_publish) {
    current_target++;
    if (current_target != path.end() && do_publish) {
        publish_pose(*current_target);
    }
}

void PathFollower::publish_pose(const tf::Pose& tp, bool do_throttle) {
    ros::Time now = ros::Time::now();
    if (!do_throttle || now - last_wp_pub >= ros::Duration(wp_throttle)) {
        geometry_msgs::Pose pose;
        tf::poseTFToMsg(tp, pose);
        wp_pub.publish(pose);
        last_wp_pub = now;
    }
}

void PathFollower::publish_target_reached() {
    TargetReached msg;
    msg.header.stamp = ros::Time::now();
    msg.path_id = path_id;
    msg.target = current_target - path.begin() - 1;
    tgt_pub.publish(msg);
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
