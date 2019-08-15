#ifndef __TB_BASE_INTERFACE_H_
#define __TB_BASE_INTERFACE_H_

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include <tb_simulation/PlanPath.h>

namespace tb_interfaces {

constexpr const char* PLANNER_SRV = "/plan_path";
constexpr const char* POSE_TOPIC = "/pose";

class BaseInterface {
public:
    BaseInterface(ros::NodeHandle nh, ros::NodeHandle nh_private);

protected:
    virtual bool plan_cb(tb_simulation::PlanPath::Request&,
                         tb_simulation::PlanPath::Response& res) = 0;
    virtual void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);

    ros::NodeHandle nh;
    ros::NodeHandle nh_private;

    ros::ServiceServer planner;
    ros::Subscriber pose_sub;

    geometry_msgs::Pose pose;
};

} // namespace tb_interfaces

#endif // __TB_BASE_INTERFACE_H_
