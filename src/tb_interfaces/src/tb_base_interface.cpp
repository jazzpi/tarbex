#include "tb_base_interface.hpp"

namespace tb_interfaces {

BaseInterface::BaseInterface(ros::NodeHandle nh, ros::NodeHandle nh_private)
    : nh{nh},
      nh_private{nh_private}
{
    planner = nh.advertiseService(PLANNER_SRV, &BaseInterface::plan_cb, this);
    pose_sub = nh.subscribe(POSE_TOPIC, 10, &BaseInterface::pose_cb, this);
}

void BaseInterface::pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    pose = msg->pose;
}

} // namespace tb_interfaces
