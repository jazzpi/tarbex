#ifndef __TB_ADSTAR_INTERFACE_H_
#define __TB_ADSTAR_INTERFACE_H_

#include "tb_2d_interface.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wexceptions"
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wunused-private-field"
#pragma GCC diagnostic ignored "-Wmismatched-tags"
#pragma GCC diagnostic ignored "-Woverloaded-virtual"
#include <sbpl/headers.h>
#pragma GCC diagnostic pop

namespace tb_interfaces {

inline uint8_t occ_grid_to_sbpl(int8_t val) {
    return (val == -1) ? 0 : val;
}

class ADStarInterface : Interface2D {
public:
    ADStarInterface(ros::NodeHandle nh, ros::NodeHandle nh_private);

protected:
    bool plan_cb(tb_simulation::PlanPath::Request& req,
                 tb_simulation::PlanPath::Response& res);

    std::vector<sbpl_2Dpt_t> robot_perimeters;
    EnvironmentNAVXYTHETALAT env;

    std::vector<unsigned char> get_mapdata();
};

}

#endif // __TB_ADSTAR_INTERFACE_H_
