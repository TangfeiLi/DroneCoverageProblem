
#include <numeric>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <fstream>
#include <limits>
#include <utility>
#include <chrono>
#include <iomanip>
#include <ctime>

#include "bb_tree.h"

namespace drone_cover {
    BBTree::BBTree(const std::string &program_params_file_name, const std::string &result_name, const std::string &data_file_name) : 
                    instance_file_name{data_file_name}, result_name{result_name} {
        prob = std::make_shared<Problem>(program_params_file_name, data_file_name);
        ub = std::numeric_limits<double>::min();
        lb = std::numeric_limits<double>::min();

        Column dummy(prob);
        dummy.make_dummy();
        pool = std::make_shared<ColumnPool>();
        pool->push_back(dummy);

        auto root_node = std::make_shared<BBNode>(prob, pool, *pool);

        unexplored_nodes.push(root_node);

        node_attaining_lb = root_node;
        node_bound_type = BoundType::FROM_LP;
        linear_optimal_state_at_root = "Linear_Optimal_At_Root";
        bb_nodes_generated = 1;
        elapsed_time = 0;
        max_depth = 0;
        total_time_on_master = 0;
        total_time_on_pricing = 0;
    }

    void BBTree::update_ub(std::shared_ptr<BBNode> current_node, unsigned int node_number) {
        // Root node. Global UB is node's UB.
        if(node_number <= 1u) {
            ub = current_node->sol_value;
            return;
        }

        // Ran out of nodes => Tree explored completely => Solution found! => LB = UB
        // if(unexplored_nodes.empty()) {
        //     lb = ub;
        //     return;
        // }

        // lb = *unexplored_nodes.top()->father_lb;
    }

    void BBTree::explore_tree() {
        using namespace std::chrono;

        print_header();

        auto node_number = 0u;
        auto start_time = high_resolution_clock::now();

        while(!unexplored_nodes.empty()) {
            std::cerr << "Nodes in tree: " << unexplored_nodes.size() << std::endl;

            auto current_node = unexplored_nodes.top();
            unexplored_nodes.pop();

            current_node->solve(node_number++);
            std::cerr << "\tNode UB: " << std::setprecision(std::numeric_limits<double>::max_digits10)
                      << current_node->sol_value << std::endl;

            if(current_node->depth > max_depth) { max_depth = current_node->depth; }

            if(current_node->sol_value <= lb) {
                std::cerr << "\t\tPruned by sub-optimality (LB = " << lb << ")" << std::endl;

                update_ub(current_node, node_number);
                gap = std::abs((ub - lb) / ub) * 100;

                continue;
            }

            try_to_obtain_lb(current_node);

            // if(current_node->has_fractional_solution() || current_node->has_solution_with_cycles()) {
            //     branch(current_node);
            // };

            update_ub(current_node, node_number);

            auto gap_node = std::abs((ub - current_node->sol_value) / ub) * 100; //节点上的gap
            gap = std::abs((ub - lb) / ub) * 100; //全局gap

            if (ub-lb<prob->params.ppm){
                std::cerr<<"ub < lb, wrong!!!"<<std::endl;
                // throw std::runtime_error("ub < lb, gets wrong!");
            }

            if(node_number == 1u) { 
                gap_at_root = gap_node; 
                if (current_node->state == StateType::SP_HEURISTIC){
                    linear_optimal_state_at_root = "Linear_Feasible_At_Root";
                }
            }

            total_time_on_master += current_node->total_time_spent_on_mp;
            total_time_on_pricing += current_node->total_time_spent_on_sp;

            print_row(*current_node, gap_node);

            auto curr_time = high_resolution_clock::now();
            auto el_time = duration_cast<duration<double>>(curr_time - start_time).count();

            if(el_time > prob->params.time_limit_in_s) {
                std::cerr << std::endl << "Over time limit! " << el_time << std::endl;
                break;
            }
        }

        auto end_time = high_resolution_clock::now();
        elapsed_time = duration_cast<duration<double>>(end_time - start_time).count();

        check_lp_ip_solution();

        print_summary();
        print_results();
    }

    void BBTree::try_to_obtain_lb(std::shared_ptr<BBNode> current_node) {
        ColumnPool feasible_columns = current_node->local_pool;

        if(current_node->has_fractional_solution()) {
            if(
                current_node->depth == 0 ||
                feasible_columns.size() <= (unsigned int) prob->params.max_cols_to_solve_mp
            ) {
                if(current_node->solve_integer(feasible_columns)) {
                    std::cerr << "\tNode LB: " << std::setprecision(std::numeric_limits<double>::max_digits10)
                              << current_node->mip_sol_value << std::endl;
                    if(current_node->mip_sol_value - lb > BBNode::cplex_epsilon) {
                        std::cerr << "\t\tImproving the LB" << std::endl;
                        lb = current_node->mip_sol_value;
                        node_attaining_lb = current_node;
                        node_bound_type = BoundType::FROM_MIP;
                        // for(const auto& cc : node_attaining_ub->mip_base_columns) {
                        //     std::cerr << "\t\t\t " << cc.first << " with coeff " << cc.second << std::endl;
                        // }
                    }
                } else {
                    std::cerr << "\tMIP infeasible" << std::endl;
                }
            } else {
                std::cerr << "\tToo many columns to solve MIP" << std::endl;
            }
        } else {
            // if(std::all_of(current_node->base_columns.begin(), current_node->base_columns.end(),
            //     [&] (const auto& cc) -> bool { return !cc.first.dummy && !cc.first.has_cycles(); }
            // )) {
            if(current_node->sol_value - lb > BBNode::cplex_epsilon) {
                std::cerr << "\t\tImproving the LB" << std::endl;
                lb = current_node->sol_value;
                node_attaining_lb = current_node;
                node_bound_type = BoundType::FROM_LP;
                // for(const auto& cc : node_attaining_ub->base_columns) {
                //     std::cerr << "\t\t\t " << cc.first << " with coeff " << cc.second << std::endl;
                // }
            }
            // }
        }
    }

    void BBTree::print_header() const {
        std::cout << std::setw(22) << "BB Nodes |";
        std::cout << std::setw(28) << "Lower Bound |";
        std::cout << std::setw(16) << "Upper Bound |";
        std::cout << std::setw(28) << "Gap |";
        std::cout << std::setw(10) << "|";
        std::cout << std::setw(66) << "Total time |";
        std::cout << std::setw(22) << "|";
        std::cout << std::setw(26) << "|";
        std::cout << std::setw(8) << "|" << std::endl;

        std::cout << std::setw(12) << "Unexplored  "; // Unexplored nodes
        std::cout << std::setw(10) << "Total |"; // Number of nodes in total

        std::cout << std::setw(14) << "LB at node  ";
        std::cout << std::setw(14) << "LB best |";

        std::cout << std::setw(16) << "UB best |";

        std::cout << std::setw(14) << "Gap at node  ";
        std::cout << std::setw(14) << "Gap best |";

        std::cout << std::setw(10) << "Columns |";

        std::cout << std::setw(22) << "Time spent on MP  "; // Time on MP
        std::cout << std::setw(22) << "Time spent on SP  "; // Time on SP
        std::cout << std::setw(22) << "Time spent at node |"; // Time at node

        std::cout << std::setw(22) << "Average time on SP |"; // Avg time on SP
        std::cout << std::setw(26) << "Max time on SP (exact) |"; // Max time on Exact SP

        std::cout << std::setw(8) << "Depth |" << std::endl;

        std::cout << "-----------*---------*" <<
                  "-------------*-------------*" <<
                  "---------------*" <<
                  "-------------*-------------*" <<
                  "---------*" <<
                  "---------------------*---------------------*---------------------*" <<
                  "---------------------*-------------------------*" <<
                  "-------*" << std::endl;
    }

    // LIBSTD of GCC DOES NOT IMPLEMENT STD::DEFAULTFLOAT !!
    inline std::ostream &defaultfloat(std::ostream &os) {
        os.unsetf(std::ios_base::floatfield);
        return os;
    }

    void BBTree::print_results() const {
        std::ofstream results_file;
        // results_file.open("results.txt", std::ios::out | std::ios::app);
        results_file.open(result_name, std::ios::out | std::ios::app);

        auto elements = std::vector<std::string>();
        auto ss = std::stringstream(instance_file_name);
        auto item = std::string();

        while(std::getline(ss, item, '/')) {
            elements.push_back(item);
        }

        // Remove .txt
        for(auto i = 0u; i < 4u; ++i) { elements.back().pop_back(); }

        ss = std::stringstream(std::string(elements.back()));
        elements = std::vector<std::string>();
        item = std::string();

        while(std::getline(ss, item, '_')) {
            elements.push_back(item);
        }

        assert(elements.size() == 7u);

        // Scenario name
        results_file << elements[0] << "_";
        // Target num
        results_file << elements[1] << "_";
        // horizons
        results_file << elements[2] << "_";
        // numclassDrone
        results_file << elements[3] << "_";
        // numDrone
        results_file << elements[4] << "_";
        // speed
        results_file << elements[5] << "_";
        // cover_range
        results_file << elements[6] << ",";

        results_file << prob->params.max_cols_to_solve_mp << ",";

        results_file << elapsed_time << ",";
        results_file << total_time_on_master << ",";
        results_file << total_time_on_pricing << ",";
        results_file << std::setprecision(12) << ub << ",";
        results_file << std::setprecision(12) << lb << ",";
        results_file << std::setprecision(12) << gap << ",";
        results_file << std::setprecision(12) << gap_at_root << ",";
        results_file << bb_nodes_generated << ",";
        results_file << max_depth << ",";
        results_file << pool->size() << ",";
        results_file << linear_optimal_state_at_root << ",";
        results_file << (node_bound_type == BoundType::FROM_LP?"From_LP":"From_MIP"); //整数解的来源

        results_file << std::endl;
        results_file.close();
    }

    void BBTree::print_row(const BBNode &current_node, double gap_node) const {
        auto print_ub = (ub < std::numeric_limits<double>::max() - 100);

        std::cout << std::fixed;
        std::cout << std::setw(10) << unexplored_nodes.size() << "  ";
        std::cout << std::setw(8) << bb_nodes_generated << "  ";
        std::cout << std::setw(12) << std::setprecision(2) << current_node.sol_value << "  ";
        std::cout << std::setw(12) << std::setprecision(2) << lb << "  ";
        if(print_ub) {
            std::cout << std::setw(14) << std::setprecision(2) << ub << "  ";
        } else {
            std::cout << std::setw(14) << "inf" << "  ";
        }
        std::cout << std::setw(11) << std::setprecision(4) << gap_node << "\%  ";
        std::cout << std::setw(11) << std::setprecision(4) << gap << "\%  ";
        std::cout << std::setw(8) << pool->size() << "  ";
        std::cout << std::setw(20) << std::setprecision(4) << current_node.total_time_spent_on_mp << "  ";
        std::cout << std::setw(20) << std::setprecision(4) << current_node.total_time_spent_on_sp << "  ";
        std::cout << std::setw(20) << std::setprecision(4) << current_node.total_time_spent << "  ";
        std::cout << std::setw(20) << std::setprecision(4) << current_node.avg_time_spent_on_sp << "  ";
        std::cout << std::setw(24) << std::setprecision(4) << current_node.max_time_spent_by_exact_solver << "  ";
        std::cout << std::setw(6) << current_node.depth << "  " << std::endl;
        defaultfloat(std::cout);
    }

    void BBTree::print_summary() const {
        std::cout << std::endl << "*** SOLUTION ***" << std::endl;
        std::cout << "Total cost: " << lb << std::endl;
        std::cout << "Linear solution state at root: "<< linear_optimal_state_at_root<<std::endl<<std::endl;

        std::cout << "*** Integer Solution Obtained from Root Node ***" << std::endl;
        std::cout << "ip cost: " << node_attaining_lb->mip_sol_value << std::endl;
        auto using_columns_per_timepoint = std::vector<double>(prob->data.num_times, 0);
        if(node_bound_type == BoundType::FROM_LP) {
            // UB was attained as LP solution
            std::cout << "*** OBTAINED FROM LP ***" << std::endl;
            std::cout<<"Number of columns = "<< node_attaining_lb->base_columns.size()<<std::endl;
            for(const auto &cc : node_attaining_lb->base_columns) {
                std::cout << "Value = " << cc.second << std::endl;
                cc.first.sol.print(std::cout, false);
                
                for(auto i = 0u; i<prob->data.num_times; ++i){
                    using_columns_per_timepoint[i] += cc.second*cc.first.sol.used_timepoints[i];
                }
            }
        } else {
            // UB was attained as MIP solution
            std::cout << "*** OBTAINED FROM MIP ***" << std::endl;
            std::cout<<"Number of columns = "<< node_attaining_lb->mip_base_columns.size()<<std::endl;
            for(const auto &cc : node_attaining_lb->mip_base_columns) {
                std::cout << "Value = " << cc.second << std::endl;
                cc.first.sol.print(std::cout, false);

                for(auto i = 0u; i<prob->data.num_times; ++i){
                    using_columns_per_timepoint[i] += cc.second*cc.first.sol.used_timepoints[i];
                }
            }
        }

        /*  timepoints used by all columns*/
        std::cout << "*** Timepoints Used by All Columns ***" << std::endl;
        for(auto i: using_columns_per_timepoint){
            std::cout<<i<<"\t";
        }
        std::cout<<std::endl;
        
        /*  LB was attained as Linear Solution*/
        std::cout << "*** Linear Solution Obtained from Root Node ***" << std::endl;
        std::cout << "lp cost: " << node_attaining_lb->sol_value << std::endl;
        std::cout<<"Number of columns = "<< node_attaining_lb->lp_base_columns.size()<<std::endl;   //注意如果后续使用BP的话，这里逻辑需要修改
        
        using_columns_per_timepoint = std::vector<double>(prob->data.num_times, 0);
        for(const auto &cc : node_attaining_lb->lp_base_columns) {
            std::cout << "Value = " << cc.second << std::endl;
            cc.first.sol.print(std::cout, false);
            
            for(auto i = 0u; i<prob->data.num_times; ++i){
                using_columns_per_timepoint[i] += cc.second*cc.first.sol.used_timepoints[i];
            }
        }
        
        /*  timepoints used by all columns*/
        std::cout << "*** Timepoints Used by All Columns ***" << std::endl;
        for(auto i: using_columns_per_timepoint){
            std::cout<<i<<"\t";
        }
        std::cout<<std::endl;
        
    }

    void BBTree::check_lp_ip_solution(){
        auto mp_solv = MPSolver(prob);
        auto lp_sol = mp_solv.solve_lp(*pool);
        auto ip_sol = mp_solv.solve_mip(*pool);

        auto using_columns_per_timepoint = std::vector<double>(prob->data.num_times, 0);

        std::cerr << "*** Check IP&LP solution ***" << std::endl;
        std::cerr << "*** Integer Solution Obtained from Root Node ***" << std::endl;
        std::cerr << "ip cost: " << ip_sol.obj_value << std::endl;
        for(auto i = 0u; i < ip_sol.route_var.size(); i++) {
            if(ip_sol.route_var[i] > BBNode::cplex_epsilon) {
                std::cerr << "Value = " << ip_sol.route_var[i] << std::endl;
                (*pool)[i].sol.print(std::cerr, false);
                
                for(auto i = 0u; i<prob->data.num_times; ++i){
                    using_columns_per_timepoint[i] += ip_sol.route_var[i]*(*pool)[i].sol.used_timepoints[i];
                }
            }
        }
        std::cerr << "*** Timepoints Used by All Columns ***" << std::endl;
        for(auto i: using_columns_per_timepoint){
            std::cerr<<i<<"\t";
        }
        std::cerr<<std::endl;

        std::cerr << "*** Linear Solution Obtained from Root Node ***" << std::endl;
        std::cerr << "lp cost: " << lp_sol.obj_value << std::endl;
        using_columns_per_timepoint = std::vector<double>(prob->data.num_times, 0);
        for(auto i = 0u; i < lp_sol.route_var.size(); i++) {
            if(lp_sol.route_var[i] > BBNode::cplex_epsilon) {
                std::cerr << "Value = " << lp_sol.route_var[i] << std::endl;
                (*pool)[i].sol.print(std::cerr, false);
                
                for(auto i = 0u; i<prob->data.num_times; ++i){
                    using_columns_per_timepoint[i] += lp_sol.route_var[i]*(*pool)[i].sol.used_timepoints[i];
                }
            }
        }
        std::cerr << "*** Timepoints Used by All Columns ***" << std::endl;
        for(auto i: using_columns_per_timepoint){
            std::cerr<<i<<"\t";
        }
        std::cerr<<std::endl;

    }
}
