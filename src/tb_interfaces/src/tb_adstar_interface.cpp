#include "tb_adstar_interface.hpp"

namespace tb_interfaces {

ADStarInterface::ADStarInterface(ros::NodeHandle nh, ros::NodeHandle nh_private)
    : Interface2D(nh, nh_private)
{
    double w_half = 0.25;
    double l_half = 0.25;

    sbpl_2Dpt_t pt;
    pt.x = -l_half;
    pt.y = -w_half;
    robot_perimeters.push_back(pt);
    pt.x = l_half;
    pt.y = -w_half;
    robot_perimeters.push_back(pt);
    pt.x = l_half;
    pt.y = w_half;
    robot_perimeters.push_back(pt);
    pt.x = -l_half;
    pt.y = w_half;
    robot_perimeters.push_back(pt);

    ROS_INFO("map topic: %s", map_sub.getTopic().c_str());
    ROS_INFO("publishers: %u", map_sub.getNumPublishers());
}

bool ADStarInterface::plan_cb(tb_simulation::PlanPath::Request& req,
                              tb_simulation::PlanPath::Response& res) {
    auto mapdata = get_mapdata();
    auto start = calc_2d_coords(pose);
    auto goal = calc_2d_coords(req.target);
    MDPConfig MDPCfg;
    env = {};
    if (!env.InitializeEnv(
            map.info.width, map.info.height, &mapdata[0], std::get<0>(start),
            std::get<1>(start), std::get<2>(start), std::get<0>(goal),
            std::get<1>(goal), std::get<2>(goal), 0.2, 0.2, 0.05,
            robot_perimeters, 0.4, 0.4, 1.0, 255,
            "/home/jasper/tarbex/src/tb_simulation/data/prim040.mprim"
        )) {
      ROS_ERROR("Couldn't initialize environment!");
      return false;
    }

    if (!env.InitializeMDPCfg(&MDPCfg)) {
      ROS_ERROR("Couldn't initialize MDPCfg!");
      return false;
    }

    ADPlanner planner(&env, 0);
    std::vector<int> solution_states;
    if (planner.set_start(MDPCfg.startstateid) == 0) {
        printf("ERROR: failed to set start state\n");
        throw SBPL_Exception("ERROR: failed to set start state");
    }
    if (planner.set_goal(MDPCfg.goalstateid) == 0) {
        printf("ERROR: failed to set goal state\n");
        throw SBPL_Exception("ERROR: failed to set goal state");
    }
    planner.set_initialsolution_eps(3.0);
    planner.set_search_mode(false);

    ROS_INFO("Start planning...");
    int ret = planner.replan(3.0, &solution_states);
    ROS_INFO("Done: %d! Size of solution = %zu", ret, solution_states.size());
    return true;
}

std::vector<unsigned char> ADStarInterface::get_mapdata() {
    int width = map.info.width;
    int height = map.info.height;
    std::vector<unsigned char> data(width * height);

    for (int x = 0; x < width; x++) {
        for (int y = 0; y < height; y++) {
            data[x + y * width] = occ_grid_to_sbpl(map.data[x + y * width]);
        }
    }

    return data;
}

} // namespace tb_interfaces

int main(int argc, char** argv) {
    ros::init(argc, argv, "adstarPlanner");

    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    tb_interfaces::ADStarInterface adstar(nh, nh_private);

    ros::spin();

    return 0;
}
