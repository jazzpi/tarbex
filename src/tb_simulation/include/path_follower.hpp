#ifndef __PATH_FOLLOWER_H_
#define __PATH_FOLLOWER_H_

#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>

namespace tb_simulation {

constexpr const char* START_SRV = "/start_planner";
constexpr const char* PLANNER_SRV = "/plan_path";
constexpr const char* WAYPOINT_TOPIC = "/waypoint";
constexpr const char* POSE_TOPIC = "/pose";
constexpr const char* FRAME_ID = "map";
constexpr const double PATH_ORIG_EPSILON = 0.1;
constexpr const double PATH_YAW_EPSILON = 0.05;

class PathFollower {
public:
    PathFollower(ros::NodeHandle nh, ros::NodeHandle nh_private);
protected:
    ros::NodeHandle nh;
    ros::NodeHandle nh_private;

    ros::ServiceServer start;
    ros::ServiceClient planner;
    ros::Publisher wp_pub;
    ros::Subscriber pose_sub;

    std::vector<tf::Pose> path;
    std::vector<tf::Pose>::const_iterator current_target;

    bool ready;
    uint32_t plan_seq;

    bool start_callback(std_srvs::Trigger::Request& req,
                        std_srvs::Trigger::Response& res);
    void pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg);

    void plan();
    void advance_target();
    void publish_pose(const tf::Pose& tp);
};

} // namespace tb_simulation

#endif // __PATH_FOLLOWER_H_
