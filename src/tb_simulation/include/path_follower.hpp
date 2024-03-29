#ifndef __PATH_FOLLOWER_H_
#define __PATH_FOLLOWER_H_

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/transform_datatypes.h>

namespace tb_simulation {

constexpr const char* WAYPOINT_TOPIC = "waypoint";
constexpr const char* TGT_REACHED_TOPIC = "target_reached";
constexpr const char* POSE_TOPIC = "pose";
constexpr const char* PATH_TOPIC = "path";
constexpr const char* FRAME_ID = "map";

class PathFollower {
public:
    PathFollower(ros::NodeHandle nh, ros::NodeHandle nh_private);
protected:
    ros::NodeHandle nh;
    ros::NodeHandle nh_private;

    double path_orig_epsilon;
    double path_yaw_epsilon;
    double wp_throttle;

    ros::Publisher wp_pub;
    ros::Publisher tgt_pub;
    ros::Subscriber pose_sub;
    ros::Subscriber path_sub;

    std::vector<tf::Pose> path;
    std::vector<tf::Pose>::const_iterator current_target;
    uint32_t path_id;

    bool ready;
    ros::Time last_wp_pub;

    void init_params();

    void pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void path_callback(const geometry_msgs::PoseArray::ConstPtr& msg);

    void advance_target(bool do_publish = true);
    void publish_pose(const tf::Pose& tp, bool do_throttle = false);
    void publish_target_reached();
};

} // namespace tb_simulation

#endif // __PATH_FOLLOWER_H_
