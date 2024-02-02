
#include "bb_node.h"

#include <algorithm>
#include <limits>
#include <numeric>
#include <iomanip>
#include <chrono>

namespace drone_cover {
    BBNode::BBNode( std::shared_ptr<const Problem> prob,
                    std::shared_ptr<ColumnPool> pool,
                    const ColumnPool &local_pool,
                    int depth,
                    StateType state,
                    std::string name,
                    bool try_elementary,
                    double avg_time_spent_on_sp,
                    double total_time_spent_on_sp,
                    double total_time_spent_on_mp,
                    double total_time_spent,
                    double max_time_spent_by_exact_solver
                ) :
                   prob(prob),
                   pool(pool),
                   local_pool(local_pool),
                   depth(depth),
                   state(state),
                   name(name),
                   try_elementary(try_elementary),
                   avg_time_spent_on_sp(avg_time_spent_on_sp),
                   total_time_spent_on_sp(total_time_spent_on_sp),
                   total_time_spent_on_mp(total_time_spent_on_mp),
                   total_time_spent(total_time_spent),
                   max_time_spent_by_exact_solver(max_time_spent_by_exact_solver)
    {
        sol_value = std::numeric_limits<double>::max();
        mip_sol_value = std::numeric_limits<double>::max();
        all_times_spent_on_sp = std::vector<double>(0);
    }

    BBNode::BBNode(const BBNode& father, std::string name) :
        prob{father.prob},
        pool{father.pool},
        local_pool{father.local_pool},
        depth{father.depth + 1},
        name{name}
    {
        sol_value = std::numeric_limits<double>::max();
        mip_sol_value = std::numeric_limits<double>::max();
        all_times_spent_on_sp = std::vector<double>(0);
    }

    // void BBNode::remove_duplicate_columns() {
    //     ColumnPool new_pool;

    //     for(const auto &c : *pool) {
    //         auto nc = std::find_if(new_pool.begin(), new_pool.end(),
    //                                [&](const Column &c_in_new_pool) {
    //                                    return column_coefficients(c) == column_coefficients(c_in_new_pool);
    //                                });

    //         if(nc == new_pool.end()) { new_pool.push_back(c); }
    //         else if(nc->obj_coeff > c.obj_coeff) { *nc = c; }
    //     }

    //     std::cerr << "\tRemoved " << pool->size() - new_pool.size() << " duplicate columns"
    //               << " out of " << pool->size() << std::endl;

    //     *pool = new_pool;
    // }

    void BBNode::solve(unsigned int node_number) {
        using namespace std::chrono;

        auto node_start = high_resolution_clock::now();

        // Clear any eventual previous solutions
        base_columns = std::vector<std::pair<Column, double>>();
        lp_base_columns = std::vector<std::pair<Column, double>>();
        sol_value = 0;

        auto mp_solv = MPSolver(prob);

        // We start by solving the LP relaxation of the Master Problem
        auto mp_start = high_resolution_clock::now();
        auto sol = mp_solv.solve_lp(local_pool);
        auto mp_end = high_resolution_clock::now();

        auto mp_time = duration_cast<duration<double>>(mp_end - mp_start).count();
        total_time_spent_on_mp += mp_time;

        std::cerr << std::unitbuf << "MP: " << std::setprecision(std::numeric_limits<double>::max_digits10)
                  << sol.obj_value << " in " << std::setprecision(2) << mp_time << " seconds" << std::endl;

        auto node_explored = false;

        // Keep going until we can prove the node has been explored!
        while(!node_explored) {
            // Update reduced cost of decision variables
            for(auto i = 0; i<local_pool.size(); ++i){
                local_pool[i].reduced_cost = sol.route_reduced_costs[i];
            }

            // Update dual values
            for(const auto &dg : prob->graphs) {
                dg.second->graph_dual_properties.inspection_duals = sol.inspection_duals;
                dg.second->graph_dual_properties.timepoint_duals = sol.timepoint_duals.at(dg.first->id);
                // dg.second->graph_dual_properties.print();
            }

            auto sp_solv = SPSolver(prob);
            auto sp_found_columns = 0;
            auto is_exact_sp = true;
            std::pair<int,bool> sp_solve_result;

            // Solve the pricing subproblem
            auto sp_start = high_resolution_clock::now();
            sp_solve_result = sp_solv.solve(local_pool, pool, max_time_spent_by_exact_solver);
            sp_found_columns = sp_solve_result.first; is_exact_sp = sp_solve_result.second;
            auto sp_end = high_resolution_clock::now();

            auto sp_time = duration_cast<duration<double>>(sp_end - sp_start).count();
            total_time_spent_on_sp += sp_time;
            all_times_spent_on_sp.push_back(sp_time);

            // for(const auto &dg : prob->graphs) {
            //     for(auto column: local_pool){
            //         if (column.sol.g == dg.second){
            //             check_column_reduced_cost(column, dg.second->graph_dual_properties);
            //         }
            //     }
            // }

            std::cerr << std::unitbuf << "SP found " << sp_found_columns << " columns in "
                      << std::setprecision(2) << sp_time << " seconds" << std::endl;

            auto node_end = high_resolution_clock::now();
            total_time_spent = duration_cast<duration<double>>(node_end - node_start).count();

            // If new [negative reduced cost] columns are found and not exceed the time_limit_in_s, solve the LP again
            if(sp_found_columns > 0 && prob->params.time_limit_in_s - total_time_spent > prob->params.ppm) {
                // Re-solve the LP
                auto mp_start = high_resolution_clock::now();
                sol = mp_solv.solve_lp(local_pool);
                auto mp_end = high_resolution_clock::now();

                total_time_spent_on_mp += duration_cast<duration<double>>(mp_end - mp_start).count();

                std::cerr << std::unitbuf << "MP: " << std::setprecision(std::numeric_limits<double>::max_digits10)
                          << sol.obj_value << std::endl;
            } else {
                // If no negative reduced cost column was found, the node is explored
                node_explored = true;

                // The [potentially fractional] solution at this node is the solution of the last LP solved
                sol_value = sol.obj_value;

                // Save the base columns of this solution
                for(auto i = 0u; i < sol.route_var.size(); i++) {
                    if(sol.route_var[i] > BBNode::cplex_epsilon) {
                        base_columns.push_back(std::make_pair(local_pool[i], sol.route_var[i]));
                        lp_base_columns.push_back(std::make_pair(local_pool[i], sol.route_var[i]));
                    }
                }

                avg_time_spent_on_sp =
                    std::accumulate(all_times_spent_on_sp.begin(), all_times_spent_on_sp.end(), 0.0) /
                    all_times_spent_on_sp.size();

                if (!is_exact_sp || total_time_spent - prob->params.time_limit_in_s > prob->params.ppm) {   //如果列生成运行时间超过预设最大时间，则没解到最优
                    state = StateType::SP_HEURISTIC;
                }else{
                    state = StateType::EXACT;
                }
            }
        }

        auto node_end = high_resolution_clock::now();
        total_time_spent = duration_cast<duration<double>>(node_end - node_start).count();
        std::cerr << "Node explored in " << std::setprecision(2) << total_time_spent << " seconds" << std::endl;

        // Remove any duplicate column we might have generated at this node
        // remove_duplicate_columns();
    }

    void BBNode::check_column_reduced_cost(const Column &column, const GraphProperties &graph_dual_properties){
        std::cerr<<"******************************************"<<std::endl;
        std::cerr<<"******************************************"<<std::endl;
        std::cerr<<"Reduced cost from cplex = "<<column.reduced_cost<<std::endl;

        double cal_reduced_cost = 0;
        auto row_num = graph_dual_properties.inspection_duals.size();
        auto col_num = graph_dual_properties.inspection_duals[0].size();
        for(auto i = 0;i<row_num; ++i){
            for(auto j = 0; j<col_num; ++j){
                cal_reduced_cost += column.sol.inspections[i][j]*graph_dual_properties.inspection_duals[i][j];
            }
        }

        for(auto i = 0; i<row_num; ++i){
            cal_reduced_cost += column.sol.used_timepoints[i]*graph_dual_properties.timepoint_duals[i];
        }
        std::cerr<<"Reduced cost from calculating = "<<-cal_reduced_cost<<std::endl;
    }

    bool BBNode::solve_integer(const ColumnPool &feasible_columns) {
        // Clear any eventual previous solutions
        mip_base_columns = std::vector<std::pair<Column, double>>();
        mip_sol_value = 0;

        MPSolver mp_solv(prob);

        // Try to solve the integer problem with the columns we have at this node
        try {
            auto sol = mp_solv.solve_mip(feasible_columns);

            // Get the objective value of the MIP
            mip_sol_value = sol.obj_value;

            // And save the base columns of the MIP solution
            for(auto i = 0u; i < sol.route_var.size(); i++) {
                if(sol.route_var[i] > BBNode::cplex_epsilon) {
                    mip_base_columns.push_back(std::make_pair(feasible_columns[i], sol.route_var[i]));
                }
            }

            return true;
        } catch(...) {
            // If a [CPLEX] exception was thrown, then it was not possible to solve the MIP... sorry!
            return false;
        }
    }

    // bool BBNode::is_feasible() const {
    //     // An LP solution is feasible if the dummy column is not in the base columns
    //     return std::none_of(base_columns.begin(), base_columns.end(),
    //                         [](const auto &cc) { return cc.first.dummy; });
    // }

    // bool BBNode::is_integer_feasible() const {
    //     // A MIP solution is feasible if the dummy column is not in the base columns
    //     return std::none_of(mip_base_columns.begin(), mip_base_columns.end(),
    //                         [](const auto &cc) { return cc.first.dummy; });
    // }

    bool BBNode::has_fractional_solution() const {
        return std::any_of(base_columns.begin(), base_columns.end(),
                           [](const auto& cc) { return cc.second > cplex_epsilon && cc.second < 1.0 - cplex_epsilon; });
    }

    // bool BBNode::has_solution_with_cycles() const {
    //     return std::any_of(base_columns.begin(), base_columns.end(),
    //                        [](const auto& cc) { return cc.first.has_cycles(); });
    // }
}
