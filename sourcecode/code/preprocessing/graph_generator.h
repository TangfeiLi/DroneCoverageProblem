
#ifndef GRAPH_GENERATOR_H
#define GRAPH_GENERATOR_H

#include <memory>

#include "../base/graph.h"
#include "../base/node.h"
#include "../base/arc.h"
#include "../base/drone_class.h"
#include "problem_data.h"
#include "program_params.h"

namespace drone_cover {
    namespace GraphGenerator {
        std::shared_ptr<Graph>
        create_graph(const ProblemData &data, std::shared_ptr<DroneClass> drone_class, Inspection_Matrix inspection_matrix);
        int cal_t(const ProblemData &data, int t, int move_time, bool is_preorder=false);

        // 递归函数，用于获取一个 Target 相邻的cover range范围内的所有相邻 Targets
        void get_all_covered_targets(const std::shared_ptr<Target>& target, std::vector<std::shared_ptr<Target>>& all_adjacent_targets, std::shared_ptr<DroneClass> drone_class, int layer);
        
        void connect_arc_with_inspection(std::shared_ptr<Arc>& arc);

        int find_arc(Arc_Vec arc_vec, std::shared_ptr<Arc> arc);

        std::vector<std::vector<int>> calculate_arc_inspections(const Arc& arc, const ProblemData &data);
        std::vector<int> calculate_arc_timepoints(const Arc& arc, const ProblemData &data);

        bool lessThanReal(const double &lhs, const double &rhs, const double &threshold);
        bool greaterThanReal(const double &lhs, const double &rhs, const double &threshold);
        bool equalToReal(const double &lhs, const double &rhs, const double &threshold);
    }
}

#endif
