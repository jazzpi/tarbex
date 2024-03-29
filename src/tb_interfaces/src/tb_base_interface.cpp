#include "tb_base_interface.hpp"

#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Duration.h>

namespace tb_interfaces {

BaseInterface::BaseInterface(ros::NodeHandle nh, ros::NodeHandle nh_private)
    : nh{nh},
      nh_private{nh_private},
      vis_seq{0},
      path_vis_id{0},
      last_path_vis_id{0},
      target_vis_id{0},
      path_id{(uint32_t) -1},
      failed_replans{0}
{
    planner = nh.advertiseService(PLANNER_SRV, &BaseInterface::plan_cb, this);
    pose_sub = nh_private.subscribe(POSE_TOPIC, 10, &BaseInterface::pose_cb, this);
    tgt_reached_sub = nh_private.subscribe(TARGET_REACHED, 10, &BaseInterface::target_reached_cb, this);
    path_pub = nh_private.advertise<geometry_msgs::PoseArray>(PATH_TOPIC, 10);
    vis_pub = nh_private.advertise<visualization_msgs::Marker>(VIS_TOPIC, 10);
    vis_arr_pub = nh_private.advertise<visualization_msgs::MarkerArray>(VIS_TOPIC + std::string("_array"), 10);
    planning_time_pub = nh.advertise<std_msgs::Duration>(PLANNING_TOPIC, 10);
    replan_timer = nh_private.createTimer(
        ros::Duration(REPLAN_TIMER_INTERVAL), &BaseInterface::replan_timer_cb,
        this);
    replan_timer.stop();
}

void BaseInterface::pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    pose = msg->pose;
}

void BaseInterface::target_reached_cb(const tb_simulation::TargetReached::ConstPtr&) {}

void BaseInterface::replan_timer_cb(const ros::TimerEvent&) {}

void BaseInterface::publish_pose(const geometry_msgs::Pose &pose) {
    publish_pose(pose, ros::Time::now());
}

void BaseInterface::publish_pose(const geometry_msgs::Pose &pose, ros::Time stamp) {
  vis_pub.publish(pose_marker(pose, stamp));
}

void BaseInterface::publish_path(const std::vector<geometry_msgs::Pose> &path) {
    ros::Time stamp = ros::Time::now();

    geometry_msgs::PoseArray path_msg;
    path_msg.header.frame_id = "/map";
    path_msg.header.stamp = stamp;
    path_msg.header.seq = ++path_id;
    path_msg.poses = path;
    path_pub.publish(path_msg);
}

void BaseInterface::publish_path_vis(const std::vector<geometry_msgs::Pose>& path,
                                     int color, bool delete_previous) {
    if (delete_previous) {
        delete_poses();
    }
    visualization_msgs::MarkerArray array;
    ros::Time stamp = ros::Time::now();
    for (const auto& p : path) {
        array.markers.push_back(pose_marker(p, stamp, color));
    }

    vis_arr_pub.publish(array);
}

void BaseInterface::publish_target_vis(const geometry_msgs::Pose& tgt) {
    visualization_msgs::Marker p;
    p.header.stamp = ros::Time::now();
    p.header.seq = vis_seq++;
    p.header.frame_id = "/map";
    p.id = target_vis_id++;
    p.ns = "target";
    p.type = visualization_msgs::Marker::LINE_LIST;
    p.action = visualization_msgs::Marker::DELETE;
    vis_pub.publish(p);

    p.header.seq = vis_seq++;
    p.id = target_vis_id;
    p.action = visualization_msgs::Marker::ADD;
    p.pose = tgt;
    p.points.clear();
    geometry_msgs::Point point;
    for (int i = 1; i <= 4; i++) {
        point.x = (i % 2 == 0) ? -0.2 : 0.2;
        point.y = (i % 4 <= 1) ? 0.2 : -0.2;
        p.points.push_back(point);
    }
    p.scale.x = p.scale.y = 0.1;
    p.color.g = 1;
    p.color.a = 1;
    vis_pub.publish(p);
}

void BaseInterface::delete_poses() {
    visualization_msgs::Marker p;
    p.header.stamp = ros::Time::now();
    p.header.seq = vis_seq++;
    p.header.frame_id = "/map";
    p.ns = "planner";
    p.type = visualization_msgs::Marker::ARROW;
    p.action = visualization_msgs::Marker::DELETE;

    visualization_msgs::MarkerArray msg;

    for (uint32_t id = last_path_vis_id; id < path_vis_id; id++) {
        p.header.seq = vis_seq++;
        p.id = id;
        msg.markers.push_back(p);
    }

    vis_arr_pub.publish(msg);

    last_path_vis_id = path_vis_id;
}

tf::Vector3 BaseInterface::normalized_dir(const geometry_msgs::Pose& p1,
                                          const geometry_msgs::Pose& p2) {
    tf::Vector3 dir;
    tf::pointMsgToTF(p1.position, dir);
    tf::Vector3 p2_;
    tf::pointMsgToTF(p2.position, p2_);
    dir -= p2_;
    return dir.normalize();
}

void BaseInterface::planning_start() {
    planning_started = ros::Time::now();
}

void BaseInterface::planning_done() {
    std_msgs::Duration msg;
    msg.data = ros::Time::now() - planning_started;

    planning_time_pub.publish(msg);
}

visualization_msgs::Marker BaseInterface::pose_marker(const geometry_msgs::Pose& pose, ros::Time stamp, int color) {
    visualization_msgs::Marker p;
    p.header.stamp = stamp;
    p.header.seq = vis_seq++;
    p.header.frame_id = "/map";
    p.id = path_vis_id++;
    p.ns = "planner";
    p.type = visualization_msgs::Marker::ARROW;
    p.action = visualization_msgs::Marker::ADD;
    p.pose = pose;
    switch (color) {
    case 1:
        p.scale.x = 0.1;
        p.scale.y = 0.1;
        p.scale.z = 0.05;
        p.color.r = 167.0 / 255.0;
        p.color.g = 0.0;
        p.color.b = 0.0;
        break;
    case 2:
        p.scale.x = 0.2;
        p.scale.y = 0.1;
        p.scale.z = 0.025;
        p.color.r = 167.0 / 255.0;
        p.color.g = 64.0 / 255.0;
        p.color.b = 0.0;
        break;
    default:
        p.scale.x = 0.05;
        p.scale.y = 0.1;
        p.scale.z = 0.1;
        p.color.r = 167.0 / 255.0;
        p.color.g = 167.0 / 255.0;
        p.color.b = 0.0;
        break;
    }
    p.color.a = 1.0;
    p.frame_locked = false;

    return p;
}

} // namespace tb_interfaces
