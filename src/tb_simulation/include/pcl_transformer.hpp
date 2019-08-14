#ifndef __PCL_TRANSFORMER_H_
#define __PCL_TRANSFORMER_H_

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>

namespace tb_simulation {

constexpr const char* TOPIC_IN = "/cloud_in";
constexpr const char* TOPIC_OUT = "/cloud_out";
constexpr const char* TARGET_FRAME = "/map";

class PclTransformer {
public:
    PclTransformer(ros::NodeHandle nh, ros::NodeHandle nh_private);
protected:
    ros::NodeHandle nh;
    ros::NodeHandle nh_private;

    tf::TransformListener tf_listener;
    ros::Subscriber pcl_sub;
    ros::Publisher pcl_pub;

    void pcl_in_callback(const sensor_msgs::PointCloud2::ConstPtr& msg);
};

} // namespace tb_simulation

#endif // __PCL_TRANSFORMER_H_
