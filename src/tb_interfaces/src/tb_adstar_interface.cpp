#include "tb_adstar_interface.hpp"

namespace geometry_msgs {

bool operator==(const Point& a, const Point& b) {
    return (a.x == b.x && a.y == b.y && a.z == b.z);
}

}

namespace tb_interfaces {

ADStarInterface::ADStarInterface(ros::NodeHandle nh, ros::NodeHandle nh_private)
    : Interface2D(nh, nh_private),
      ready{false}
{
    init_params();
}

void ADStarInterface::init_params() {
    double w, l;
    nh_private.param<double>("bbx_width", w, 0.5);
    nh_private.param<double>("bbx_length", l, 0.5);
    double w_half = w / 2;
    double l_half = l / 2;

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
}

bool ADStarInterface::plan_cb(tb_simulation::PlanPath::Request& req,
                              tb_simulation::PlanPath::Response& res) {
    target = req.target;
    publish_target_vis(target);

    if (!reinit_env()) {
        return false;
    }

    ready = true;

    ROS_INFO("Publishing new path because of service call");
    res.path = publish_xytheta(replan());

    return true;
}

void ADStarInterface::target_reached_cb(const tb_simulation::TargetReached::ConstPtr& msg) {
    current_target = msg->target + 1;

    if (!ready || msg->path_id != path_id || (current_path.size() > 1 && msg->target == 0)) return;

    auto xytheta = replan();
    if (xytheta.size() == 0) {
      ROS_WARN_THROTTLE(1, "Failed to replan after reaching target!");
      // TODO: Start replan timer?
    } else if (path_changed(xytheta)) {
        ROS_INFO("Publishing new path because the target was reached and the path changed");
        publish_xytheta(xytheta);
    } else {
        ROS_INFO("Not publishing new path because it didn't change after target was reached");
    }
}

std::vector<geometry_msgs::Pose> ADStarInterface::publish_xytheta(const std::vector<sbpl_xy_theta_pt_t>& xytheta) {
    std::vector<geometry_msgs::Pose> path;

    /* Skip the first pose - we're already there */
    for (auto it = xytheta.begin() + 1; it < xytheta.end(); it++) {
        geometry_msgs::Pose po = calc_pose(std::make_tuple(it->x, it->y, it->theta));
        path.push_back(po);
    }

    current_path = xytheta;
    current_target = 0;
    publish_path(path);

    return path;
}

void ADStarInterface::pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    geometry_msgs::Pose old_pose = pose;
    BaseInterface::pose_cb(msg);

    if (!ready) return;

    auto old_2d = calc_2d_coords(old_pose);
    auto pose_2d = calc_2d_coords(pose);
    auto old_xyt = calc_discrete_coords(std::get<0>(old_2d), std::get<1>(old_2d),
                                        std::get<2>(old_2d));
    auto xyt = calc_discrete_coords(std::get<0>(pose_2d), std::get<1>(pose_2d),
                                    std::get<2>(pose_2d));
    if (std::get<0>(xyt) == std::get<0>(old_xyt) &&
        std::get<1>(xyt) == std::get<1>(old_xyt) &&
        std::get<2>(xyt) == std::get<2>(old_xyt)) {
        return;
    }

    int start_id = env.SetStart(std::get<0>(pose_2d), std::get<1>(pose_2d),
                                std::get<2>(pose_2d));
    if (start_id == -1) {
        ROS_ERROR("Couldn't update environment start");
        return;
    }
    if (!planner->set_start(start_id)) {
        ROS_ERROR("Couldn't update planner start");
        return;
    }
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

void ADStarInterface::process_map_updates(
        const std::vector<std::tuple<int, int, int8_t>> &updated_cells) {
    if (!ready) {
        return;
    }

    std::vector<nav2dcell_t> changed_cells;
    for (const auto& p : updated_cells) {
        int x = std::get<0>(p);
        int y = std::get<1>(p);
        env.UpdateCost(x, y, occ_grid_to_sbpl(std::get<2>(p)));
        changed_cells.push_back({x, y});
    }

    std::vector<int> preds;
    env.GetPredsofChangedEdges(&changed_cells, &preds);
    planner->update_preds_of_changededges(&preds);

    auto xytheta = replan();
    if (path_changed(xytheta)) {
        ROS_INFO("Publishing new path because of map update");
        publish_xytheta(xytheta);
    } else {
        ROS_INFO("Not publishing new path because it didn't change after map update");
    }
}

void ADStarInterface::process_map_replaced() {
    reinit_env();
}

std::vector<sbpl_xy_theta_pt_t> ADStarInterface::replan() {
    std::vector<sbpl_xy_theta_pt_t> xytheta;
    std::vector<int> solution_states;
    ROS_DEBUG("Start planning...");
    planning_start();
    int ret;
    try {
        ret = planner->replan(3.0, &solution_states);
    } catch (SBPL_Exception e) {
        ROS_ERROR("Planner threw an exception: %s", e.what());
        return xytheta;
    }
    if (ret) {
        ROS_DEBUG("Done! Size of solution = %zu", solution_states.size());
    } else {
        ROS_WARN("Couldn't find a solution!");
        return xytheta;
    }

    env.ConvertStateIDPathintoXYThetaPath(&solution_states, &xytheta);

    compress_path(xytheta);
    planning_done();

    return xytheta;
}

std::tuple<int, int, int> ADStarInterface::calc_discrete_coords(double x_, double y_, double theta_) {
    int x = CONTXY2DISC(x_, 0.4);
    int y = CONTXY2DISC(y_, 0.4);
    int theta = env.ContTheta2DiscNew(theta_);

    return std::make_tuple(x, y, theta);
}

bool ADStarInterface::path_changed(const std::vector<sbpl_xy_theta_pt_t>& new_path) {
    if (new_path.size() + current_target != current_path.size()) {
        return true;
    }

    // If fresh path, don't check the first taret -- we're already there anyways
    uint64_t i = (current_target == 0) ? 1 : 0;
    for (; i < new_path.size(); i++) {
        if (!(new_path[current_target + i] == current_path[i])) {
            return true;
        }
    }

    return false;
}

void ADStarInterface::compress_path(std::vector<sbpl_xy_theta_pt_t>& path) {
    if (path.size() == 0) return;

    auto last_point = path[0];
    sbpl_2Dpt_t last_dir = {NAN, NAN};
    std::vector<geometry_msgs::Pose> erased_turn;
    std::vector<geometry_msgs::Pose> erased_dir;
    std::vector<geometry_msgs::Pose> kept;
    for (size_t i = 1; i < path.size();) {
        auto p = path[i];
        sbpl_2Dpt_t dir = {p.x - last_point.x, p.y - last_point.y};
        normalize_dir(dir);
        /* We want to erase in two cases:
         *
         * - When the position doesn't change (i.e. we're turning in place),
         *   only keep the first and last one
         * - When the direction doesn't change, only keep the last one
         */
        if (dir.x == 0 && dir.y == 0 && last_dir.x == 0 && last_dir.y == 0) {
            erased_turn.push_back(calc_pose(std::make_tuple(last_point.x, last_point.y, last_point.theta)));
            path.erase(path.begin() + (i - 1));
        } else if (std::fabs(dir.x - last_dir.x) <= 0.01 &&
                   std::fabs(dir.y - last_dir.y) <= 0.01 &&
                   std::fabs(p.theta - last_point.theta) <= 0.01) {
            erased_dir.push_back(calc_pose(std::make_tuple(last_point.x, last_point.y, last_point.theta)));
            path.erase(path.begin() + (i - 1));
        } else {
            kept.push_back(calc_pose(std::make_tuple(last_point.x, last_point.y, last_point.theta)));
            i++;
        }
        last_point = p;
        last_dir = dir;
    }

    publish_path_vis(kept, 0, true);
    publish_path_vis(erased_turn, 1);
    publish_path_vis(erased_dir, 2);
}

void ADStarInterface::normalize_dir(sbpl_2Dpt_t& dir) {
    double length = std::sqrt(dir.x * dir.x + dir.y * dir.y);
    if (length == 0) return;

    dir.x /= length;
    dir.y /= length;
}

bool ADStarInterface::reinit_env() {
    auto mapdata = get_mapdata();
    auto start = calc_2d_coords(pose);
    auto goal = calc_2d_coords(target);
    env = {};
    if (!env.InitializeEnv(
            map.info.width, map.info.height, &mapdata[0], std::get<0>(start),
            std::get<1>(start), std::get<2>(start), std::get<0>(goal),
            std::get<1>(goal), std::get<2>(goal), 0.2, 0.2, 0.05,
            robot_perimeters, 0.4, 0.4, 1.0, 100,
            "/home/jasper/tarbex/src/tb_simulation/data/prim040.mprim"
        )) {
      ROS_ERROR("Couldn't initialize environment!");
      return false;
    }

    mdp_cfg = {};
    if (!env.InitializeMDPCfg(&mdp_cfg)) {
      ROS_ERROR("Couldn't initialize MDPCfg!");
      return false;
    }

    planner = std::make_unique<ADPlanner>(&env, 0);
    if (planner->set_start(mdp_cfg.startstateid) == 0) {
        printf("ERROR: failed to set start state\n");
        return false;
    }
    if (planner->set_goal(mdp_cfg.goalstateid) == 0) {
        printf("ERROR: failed to set goal state\n");
        return false;
    }
    planner->set_initialsolution_eps(3.0);
    planner->set_search_mode(false);

    return true;
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
