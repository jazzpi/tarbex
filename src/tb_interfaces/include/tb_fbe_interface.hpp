#ifndef __TB_FBE_INTERFACE_H_
#define __TB_FBE_INTERFACE_H_

#include "tb_base_interface.hpp"

namespace tb_interfaces {

constexpr const char* EXPLORATION_PATH_SRV = "/get_exploration_path";
constexpr const double DIR_DIFF_EPS = 0.05;

class FBEInterface : BaseInterface {
public:
    FBEInterface(ros::NodeHandle nh, ros::NodeHandle nh_private);

protected:
    bool plan_cb(tb_simulation::PlanPath::Request& req,
                 tb_simulation::PlanPath::Response& res) override;
    void target_reached_cb(const tb_simulation::TargetReached::ConstPtr& msg) override;
    void replan_timer_cb(const ros::TimerEvent& ev) override;
    bool replan();
    bool replan(std::vector<geometry_msgs::Pose>& path);

    ros::ServiceClient get_exploration_path;

    bool ready;
    size_t path_length;
    geometry_msgs::Pose target;
};

} // namespace tb_interfaces

#endif // __TB_FBE_INTERFACE_H_
