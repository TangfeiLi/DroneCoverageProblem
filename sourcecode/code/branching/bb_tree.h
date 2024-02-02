
#ifndef BB_TREE_H
#define BB_TREE_H

#include <memory>
#include <queue>
#include <string>
#include <vector>

#include "../base/problem.h"
#include "bb_node.h"
#include "../column/column_pool.h"

namespace drone_cover {
    enum class BoundType {
        FROM_LP, FROM_MIP
    };

    class BBTree {
        using NodeQueue = std::priority_queue<std::shared_ptr<BBNode>, std::vector<std::shared_ptr<BBNode>>, BBNodeCompare>;

    public:
        std::shared_ptr<const Problem> prob;
        std::shared_ptr<ColumnPool> pool;
        NodeQueue unexplored_nodes;
        double ub;
        double lb;
        int bb_nodes_generated;
        std::shared_ptr<BBNode> node_attaining_lb;
        BoundType node_bound_type;
        std::string linear_optimal_state_at_root;   //"Linear_Optimal_At_Root" or "Linear_Feasible_At_Root"
        std::string instance_file_name;
        std::string result_name;
        double gap_at_root;
        double gap;
        double elapsed_time;
        int max_depth;
        double total_time_on_master;
        double total_time_on_pricing;
        int max_using_columns;  //同一时刻在途的最大无人机数量

        BBTree(const std::string &program_params_file_name, const std::string &result_name, const std::string &data_file_name);

        void explore_tree();

    private:
        void branch(std::shared_ptr<BBNode> current_node);

        void try_to_obtain_lb(std::shared_ptr<BBNode> current_node);
        void print_header() const;
        void print_row(const BBNode &current_node, double gap_node) const;
        void print_summary() const;
        void print_results() const;
        void update_ub(std::shared_ptr<BBNode> current_node, unsigned int node_number);

        void check_lp_ip_solution();
    };
}

#endif