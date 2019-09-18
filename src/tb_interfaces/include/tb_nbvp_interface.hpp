#ifndef __TB_NBVP_INTERFACE_H_
#define __TB_NBVP_INTERFACE_H_

#include <tf/transform_datatypes.h>

#include "tb_base_interface.hpp"

namespace tb_interfaces {

constexpr const char* FRAME_ID = "map";
constexpr const char* NBVP_SRV = "/nbvplanner";

class NBVPInterface : BaseInterface {
public:
    NBVPInterface(ros::NodeHandle nh, ros::NodeHandle nh_private);

protected:
    bool plan_cb(tb_simulation::PlanPath::Request& req,
                 tb_simulation::PlanPath::Response& res) override;
    void target_reached_cb(const tb_simulation::TargetReached::ConstPtr& msg) override;

    ros::ServiceClient get_exploration_path;

    std::vector<tf::Pose> path;
    std::vector<tf::Pose>::const_iterator current_target;

    bool ready;
    bool planning;
    ros::Time last_plan;
    uint32_t plan_seq;
    geometry_msgs::Pose target;

    bool replan();
    bool replan(std::vector<geometry_msgs::Pose>& path);
    void advance_target();
    void publish_pose(const tf::Pose& tp);
};

} // namespace tb_interfaces

#endif // __TB_NBVP_INTERFACE_H_
