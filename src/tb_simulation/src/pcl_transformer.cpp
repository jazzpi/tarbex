#include "pcl_transformer.hpp"

#include <pcl_ros/transforms.h>

namespace tb_simulation {

PclTransformer::PclTransformer(ros::NodeHandle nh, ros::NodeHandle nh_private)
    : nh{nh},
      nh_private{nh_private},
      tf_listener{nh}
{
    pcl_sub = nh.subscribe(
        TOPIC_IN, 10, &PclTransformer::pcl_in_callback, this
    );
    pcl_pub = nh.advertise<sensor_msgs::PointCloud2>(TOPIC_OUT, 10);
}

void PclTransformer::pcl_in_callback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    /* Wait for a transform so the point cloud is integrated with the correct rotation */
    tf_listener.waitForTransform(
        TARGET_FRAME, msg->header.frame_id, msg->header.stamp, ros::Duration(3.0)
    );
    // sensor_msgs::PointCloud2 out;
    // pcl_ros::transformPointCloud(TARGET_FRAME, *msg, out, tf_listener);
    ROS_INFO_THROTTLE(5, "Got point cloud in frame %s!", msg->header.frame_id.c_str());
    pcl_pub.publish(*msg);
}

} // namespace tb_simulation

int main(int argc, char** argv) {
    ros::init(argc, argv, "pclTransformer");

    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    tb_simulation::PclTransformer transformer(nh, nh_private);

    ros::spin();
    return 0;
}
