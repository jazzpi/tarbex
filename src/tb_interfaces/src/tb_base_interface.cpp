#include "tb_base_interface.hpp"

#include <visualization_msgs/MarkerArray.h>

namespace tb_interfaces {

BaseInterface::BaseInterface(ros::NodeHandle nh, ros::NodeHandle nh_private)
    : nh{nh},
      nh_private{nh_private},
      vis_id{0}
{
    planner = nh.advertiseService(PLANNER_SRV, &BaseInterface::plan_cb, this);
    pose_sub = nh.subscribe(POSE_TOPIC, 10, &BaseInterface::pose_cb, this);
    vis_pub = nh_private.advertise<visualization_msgs::Marker>(VIS_TOPIC, 10);
    vis_arr_pub = nh_private.advertise<visualization_msgs::MarkerArray>(VIS_TOPIC + std::string("_array"), 10);
}

void BaseInterface::pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    pose = msg->pose;
}

void BaseInterface::publish_pose(const geometry_msgs::Pose &pose) {
    publish_pose(pose, ros::Time::now());
}

void BaseInterface::publish_pose(const geometry_msgs::Pose &pose, ros::Time stamp) {
  vis_pub.publish(pose_marker(pose, stamp));
}

void BaseInterface::publish_path(const std::vector<geometry_msgs::Pose> &path, bool delete_previous) {
    if (delete_previous) {
        delete_poses();
    }
    visualization_msgs::MarkerArray array;
    ros::Time stamp = ros::Time::now();
    for (const auto& p : path) {
        array.markers.push_back(pose_marker(p, stamp));
    }

    vis_arr_pub.publish(array);
}

void BaseInterface::delete_poses() {
    visualization_msgs::Marker p;
    p.header.stamp = ros::Time::now();
    p.header.seq = vis_id;
    p.header.frame_id = "/map";
    p.id = vis_id;
    vis_id++;
    p.ns = "planner";
    p.type = visualization_msgs::Marker::ARROW;
    p.action = visualization_msgs::Marker::DELETEALL;
    vis_pub.publish(p);
}

visualization_msgs::Marker BaseInterface::pose_marker(const geometry_msgs::Pose& pose, ros::Time stamp) {
    visualization_msgs::Marker p;
    p.header.stamp = stamp;
    p.header.seq = vis_id;
    p.header.frame_id = "/map";
    p.id = vis_id;
    vis_id++;
    p.ns = "planner";
    p.type = visualization_msgs::Marker::ARROW;
    p.action = visualization_msgs::Marker::ADD;
    p.pose = pose;
    p.scale.x = 0.05;
    p.scale.y = 0.1;
    p.scale.z = 0.1;
    p.color.r = 167.0 / 255.0;
    p.color.g = 167.0 / 255.0;
    p.color.b = 0.0;
    p.color.a = 1.0;
    p.lifetime = ros::Duration(10.0);
    p.frame_locked = false;

    return p;
}

} // namespace tb_interfaces
