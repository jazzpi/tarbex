#include <memory>

#include <ros/ros.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>

octomap::point3d pmin;
octomap::point3d pmax;

double calc_unknown(std::shared_ptr<octomap::OcTree> tree) {
    /* Adapted from OcTreeBaseImpl::getUnknownLeafCenters() */
    double tree_depth = tree->getTreeDepth();
    double depth = tree_depth;
    octomap::point3d pmin_clamped = tree->keyToCoord(tree->coordToKey(pmin, depth), depth);
    octomap::point3d pmax_clamped = tree->keyToCoord(tree->coordToKey(pmax, depth), depth);

    float diff[3];
    unsigned int steps[3];
    float step_size = tree->getResolution() * pow(2, tree_depth-depth);
    for (int i=0;i<3;++i) {
        diff[i] = pmax_clamped(i) - pmin_clamped(i);
        steps[i] = floor(diff[i] / step_size);
    }

    double known = 0;

    octomap::point3d p = pmin_clamped;
    octomap::OcTreeNode* res;
    for (unsigned int x=0; x<steps[0]; ++x) {
        p.x() += step_size;
        for (unsigned int y=0; y<steps[1]; ++y) {
            p.y() += step_size;
            for (unsigned int z=0; z<steps[2]; ++z) {
                //          std::cout << "querying p=" << p << std::endl;
                p.z() += step_size;
                res = tree->search(p,depth);
                if (res != NULL) {
                    known += pow(step_size, 3);
                }
            }
            p.z() = pmin_clamped.z();
        }
        p.y() = pmin_clamped.y();
    }

    double volume = (pmax.x() - pmin.x()) * (pmax.y() - pmin.y()) * (pmax.z() - pmin.z());
    ROS_INFO("%f/%f = %f known (%lu nodes)", known, volume, known / volume,
             tree->calcNumNodes());

    double minx, miny, minz, maxx, maxy, maxz;
    tree->getMetricMin(minx, miny, minz);
    tree->getMetricMax(maxx, maxy, maxz);
    ROS_INFO("Bounding Box: [%f, %f, %f] -> [%f, %f, %f]", minx, miny, minz, maxx, maxy, maxz);

    return 0;
}

void map_cb(const octomap_msgs::Octomap& msg) {
    std::shared_ptr<octomap::AbstractOcTree> tree(octomap_msgs::msgToMap(msg));
    std::string type = tree->getTreeType();
    assert(type == "OcTree");
    auto oct = std::dynamic_pointer_cast<octomap::OcTree>(tree);
    ROS_INFO("Got message with tree %s. %f unknown!",
             tree->getTreeType().c_str(), calc_unknown(oct));
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "tb_logger");

    ros::NodeHandle n;

    ros::NodeHandle params("~");
    float x, y, z;
    assert(params.getParam("bb1x", x));
    assert(params.getParam("bb1y", y));
    assert(params.getParam("bb1z", z));
    pmin = {x, y, z};
    assert(params.getParam("bb2x", x));
    assert(params.getParam("bb2y", y));
    assert(params.getParam("bb2z", z));
    pmax = {x, y, z};

    ros::Subscriber sub = n.subscribe("octomap_full", 100, map_cb);

    ros::spin();

    return 0;
}
