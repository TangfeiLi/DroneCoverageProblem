

#include <limits>
#include <iostream>

#include "solution.h"

namespace drone_cover {
    void Solution::print(std::ostream& outputStream, bool detail) const{
        if (detail){
            outputStream<<"Arc id of the Path:"<<std::endl;
            for(auto id: path){
                outputStream<<id<<"\t";
            }
            outputStream<<std::endl;
            outputStream<<"Reduced cost: "<< reduced_cost <<std::endl;
        }

        outputStream<<"Nodes of the Path: "<<std::endl;
        // outputStream<<"("<<g->arc_vec[path[0]]->left_node->t<<","<<g->arc_vec[path[0]]->left_node->loc<<")->";
        for(auto id: path){
            outputStream<<"("<<g->arc_vec[id]->left_node->t<<","<<g->arc_vec[id]->left_node->loc<<")->";
            outputStream<<"("<<g->arc_vec[id]->right_node->t<<","<<g->arc_vec[id]->right_node->loc<<")->";
        }
        outputStream<<std::endl;

        if (detail){
            for(auto i = 0; i<trajectory_nodes.size(); ++i){
                for(auto j = 0; j<trajectory_nodes[0].size(); ++j){
                    outputStream<<trajectory_nodes[i][j]<<"\t";
                }
                outputStream<<std::endl;
            }

            outputStream<<std::endl<<"Flight time = "<<flight_time<<std::endl;

            outputStream<<"Inspections: "<<std::endl;
            for(auto i = 0; i<inspections.size(); ++i){
                for(auto j = 0; j<inspections[0].size(); ++j){
                    outputStream<<inspections[i][j]<<"\t";
                }
                outputStream<<std::endl;
            }

            outputStream<<"Used_timepoints: "<<std::endl;
            for(auto i = 0; i<used_timepoints.size(); ++i){
                outputStream<<used_timepoints[i]<<"\t";
            }
            outputStream<<std::endl;
        }
    }

    // bool Solution::operator==(const Solution &other) const {
    //     if(other.drone_class != drone_class) {
    //         return false;
    //     }
    //     if(other.path.size() != path.size()) {
    //         return false;
    //     }
    //     if(fabs(cost - other.cost) > 0.000001) {
    //         return false;
    //     }
    //     for(auto i = 0u; i < path.size(); i++) {
    //         if(*other.g->graph[source(other.path[i], other.g->graph)] != *g->graph[source(path[i], g->graph)]) {
    //             return false;
    //         }
    //     }
    //     return true;
    // }

//     double Solution::length() const {
//         auto l = 0.0;

//         for(const auto &e : path) {
//             l += g->graph[e]->length;
//         }

//         return l;
//     }

//     double Solution::highest_load_efficiency() const {
//         auto highest = 0.0;
//         auto current = 0.0;

//         for(const auto &e : path) {
//             auto v = target(e, g->graph);
//             const auto& p = *g->graph[v];

//             if(p.pu_type == PortType::PICKUP) {
//                 current += p.pu_demand();
//             } else if(p.pu_type == PortType::DELIVERY) {
//                 current -= p.de_demand();
//             }

//             if(current > highest) { highest = current; }
//         }

//         return highest / vessel_class->capacity;
//     }

//     std::vector<double> Solution::cargo_travel_distances() const {
//         auto l = length();
//         auto current_distance = 0.0;
//         auto distances = std::vector<double>();

//         for(const auto &e : path) {
//             current_distance += g->graph[e]->length;

//             const Node &dest = *g->graph[target(e, g->graph)];

//             if(dest.n_type == NodeType::REGULAR_PORT) {
//                 if(dest.pu_type == PortType::PICKUP) {
//                     distances.push_back(l - current_distance);
//                 } else {
//                     distances.push_back(current_distance);
//                 }
//             }
//         }

//         assert(std::abs(current_distance - l) < 0.00001);

//         return distances;
//     }

//     std::vector<double> Solution::legs_distance() const {
//         auto dist = std::vector<double>();

//         for(const auto &e : path) {
//             dist.push_back(g->graph[e]->length);
//         }

//         return dist;
//     }

//     std::vector<double> Solution::legs_speed() const {
//         auto speeds = std::vector<double>();

//         for(const auto &e : path) {
//             auto s = source(e, g->graph);
//             auto t = target(e, g->graph);
//             auto l = g->graph[e]->length;
//             auto s_ti = g->graph[s]->time_step;
//             auto t_ti = g->graph[t]->time_step;
//             auto h = g->graph[t]->handling_time();

//             speeds.push_back(l / (t_ti - h - s_ti));
//         }

//         return speeds;
//     }

//     SolutionCosts Solution::solution_costs() const {
//         SolutionCosts s;

//         for(const auto &e : path) {
//             s.add(SolutionCosts(
//                 g->graph[e]->bunker_costs,
//                 g->graph[e]->tc_costs,
//                 g->graph[e]->port_costs,
//                 g->graph[e]->movement_costs,
//                 g->graph[e]->revenue
//             ));
//         }

//         return s;
//     }

//     bool Solution::uses_arc(Edge e) const {
//         return std::find(path.begin(), path.end(), e) != path.end();
//     }

}
