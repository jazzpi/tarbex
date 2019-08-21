#ifndef __TB_BASE_INTERFACE_H_
#define __TB_BASE_INTERFACE_H_

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include <tb_simulation/PlanPath.h>
#include <tb_simulation/TargetReached.h>
#include <visualization_msgs/Marker.h>

namespace tb_interfaces {

constexpr const char* PLANNER_SRV = "/plan_path";
constexpr const char* TARGET_REACHED = "target_reached";
constexpr const char* POSE_TOPIC = "pose";
constexpr const char* PATH_TOPIC = "path";
constexpr const char* VIS_TOPIC = "vis";

class BaseInterface {
public:
    BaseInterface(ros::NodeHandle nh, ros::NodeHandle nh_private);

protected:
    virtual bool plan_cb(tb_simulation::PlanPath::Request&,
                         tb_simulation::PlanPath::Response& res) = 0;
    virtual void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
    virtual void target_reached_cb(const tb_simulation::TargetReached::ConstPtr& msg);
    void publish_pose(const geometry_msgs::Pose& pose);
    void publish_pose(const geometry_msgs::Pose& pose, ros::Time stamp);
    void publish_path(const std::vector<geometry_msgs::Pose>& path);
    void publish_path_vis(const std::vector<geometry_msgs::Pose>& path,
                          int color = 0, bool delete_previous = false);
    void delete_poses();

    ros::NodeHandle nh;
    ros::NodeHandle nh_private;

    ros::ServiceServer planner;
    ros::Subscriber pose_sub;
    ros::Subscriber tgt_reached_sub;
    ros::Publisher path_pub;
    ros::Publisher vis_pub;
    ros::Publisher vis_arr_pub;

    geometry_msgs::Pose pose;
    uint32_t vis_id;
    uint32_t path_id;

private:
    visualization_msgs::Marker pose_marker(const geometry_msgs::Pose& pose, ros::Time stamp, int color = 0);
};

} // namespace tb_interfaces

#endif // __TB_BASE_INTERFACE_H_
