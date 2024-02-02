
#ifndef SOLUTION_H
#define SOLUTION_H

#include <memory>

#include "../base/graph.h"
#include "../base/drone_class.h"

namespace drone_cover {
    struct Solution {   //其实就是一条path
        Path path;
        double reduced_cost;    //TODO:检查下是否有用
        std::shared_ptr<DroneClass> drone_class;
        std::shared_ptr<const Graph> g;
        std::vector<std::vector<int>> inspections; //如果被 路径 cover，则为1; 否则为0
        std::vector<TimePoint> used_timepoints;    //如果时刻t起飞或者飞行，则为1; 否则为0
        std::vector<std::vector<int>> trajectory_nodes;
        int flight_time;

        Solution() {}

        Solution(Path path, double reduced_cost, std::shared_ptr<const Graph> g) :
            path{path}, inspections{g->calculate_path_inspections(path)}, used_timepoints{g->calculate_path_timepoints(path)}, trajectory_nodes{g->calculate_path_trajectory(path)}, reduced_cost{reduced_cost}, drone_class{g->drone_class}, flight_time(g->calculate_path_flighttime(path)), g{g} {}

        Solution(Path path, const std::vector<std::vector<int>> &inspections, const std::vector<TimePoint> &used_timepoints, double reduced_cost, std::shared_ptr<DroneClass> drone_class, std::shared_ptr<const Graph> g) :
            path{path}, inspections{inspections}, used_timepoints{used_timepoints}, reduced_cost{reduced_cost}, drone_class{drone_class}, g{g} {}

        bool operator==(const Solution &other) const;
        double length() const;

        void print(std::ostream& outputStream, bool detail = true) const;
    };
}
#endif