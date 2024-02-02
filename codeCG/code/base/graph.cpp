
#include <numeric>
#include <algorithm>
#include <limits>
#include <stdexcept>
#include <fstream>

#include "graph.h"

namespace drone_cover {

    std::vector<std::vector<int>> Graph::calculate_path_inspections(const Path& p) const {
        auto row_num = node_matrix.size();
        auto col_num = node_matrix[0].size();

        std::vector<std::vector<int>> result = std::vector<std::vector<int>>(row_num, std::vector<int>(col_num, 0));
        for(auto arc_id: p){
            for(int i = 0; i<row_num; ++i){
                for(int j = 0; j<col_num; ++j){
                    result[i][j] = (arc_vec[arc_id]->inspections[i][j] == 1)?1:result[i][j];
                }
            }
        }
        return result;
    }

    std::vector<TimePoint> Graph::calculate_path_timepoints(const Path& p) const{
        auto row_num = depot_nodes.size();

        auto result = std::vector<TimePoint>(row_num, 0);
        for(auto arc_id: p){
            auto t = arc_vec[arc_id]->left_node->t;
            while(t != arc_vec[arc_id]->right_node->t){
                result[t] = 1;
                t = t+1;
                t = t>=row_num?t-row_num:t;
            }
        }
        return result;
    }

    int Graph::calculate_path_flighttime(const Path& p) const {
        auto row_num = depot_nodes.size();

        int result = 0;
        for(auto arc_id: p){
            auto t = arc_vec[arc_id]->left_node->t;
            while(t != arc_vec[arc_id]->right_node->t){
                result++;
                t = t+1;
                t = t>=row_num?t-row_num:t;
            }
        }
        return result;
    }

    std::vector<std::vector<int>> Graph::calculate_path_trajectory(const Path& p) const {
        auto row_num = node_matrix.size();
        auto col_num = node_matrix[0].size();

        std::vector<std::vector<int>> result = std::vector<std::vector<int>>(row_num, std::vector<int>(col_num, 0));
        for(auto arc_id: p){
            result[arc_vec[arc_id]->left_node->t][arc_vec[arc_id]->left_node->loc] = 1;
            result[arc_vec[arc_id]->right_node->t][arc_vec[arc_id]->right_node->loc] = 1;
        }
        return result;
    }

}
