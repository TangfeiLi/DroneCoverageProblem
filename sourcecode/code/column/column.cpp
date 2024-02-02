

#include <iomanip>
#include <iostream>
#include <limits>

#include "column.h"

namespace drone_cover {
    void Column::make_dummy(){
        auto g = prob->graphs.at(prob->data.drone_classes[0]);

        /****** construct the feasible initial path *******/
        Path p = Path();
        auto start_node = g->depot_nodes[0];
        auto start_arc_id = start_node->right_arcs[0];
        p.push_back(start_arc_id);
        // g->arc_vec[start_arc_id]->print();

        for(auto arc_id: g->arc_vec[start_arc_id]->right_node->right_arcs){
            if (g->arc_vec[arc_id]->right_node->loc == 0){
                p.push_back(arc_id);
                // g->arc_vec[arc_id]->print();
                break;
            }
        }

        if (p.size() != 2){
            throw std::runtime_error("intial path cannot construct!!");
        }
        
        sol = Solution(p, 0, g);
        // sol.print();
        // auto inspections = std::vector<std::vector<int>>(prob->data.num_times, std::vector<int>(prob->data.num_targets+1, 1));
        // sol.inspections = inspections;
        // auto used_timepoints = std::vector<TimePoint>(prob->data.num_times, 1);
        // sol.used_timepoints = used_timepoints;

        dummy = true;
    }
}