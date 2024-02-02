
#include <iostream>
#include <limits>
#include <mutex>
#include <string>
#include <thread>
#include <chrono>

#include "sp_solver.h"

namespace drone_cover {

    void SPSolver::print_report(int sols_found, int discarded_prc, int discarded_infeasible, int discarded_generated,
                                int discarded_in_pool, std::ostream &out) const {
        out << "\t\t\tWe found " << sols_found << " new columns." << std::endl;
        out << "\t\t\t\t" << discarded_prc << " columns were discarded because they have positive reduced cost." << std::endl;
        out << "\t\t\t\t" << discarded_infeasible << " columns were discarded because they're infeasible wrt capacity constraints." << std::endl;
        out << "\t\t\t\t" << discarded_generated << " columns were discarded because they had already been generated in this iteration." << std::endl;
        out << "\t\t\t\t" << discarded_in_pool << " columns were discarded because they were already in the columns pool." << std::endl;
    }

    std::pair<int,bool> SPSolver::solve(ColumnPool &node_pool, std::shared_ptr<ColumnPool> global_pool, double &max_time_spent_by_exact_solver) const {
        using namespace std::chrono;

        std::vector<Solution> valid_sols;

        auto row_num = prob->data.num_times;
        auto col_num = prob->data.num_targets+1;    //包含depot
        /********************** Exact Solver **********************/

        bool is_exact = true;
        // std::cerr << "\t\tExact Solver" << std::endl;
        auto sp_start = high_resolution_clock::now();
        for(auto dcit = prob->data.drone_classes.begin(); dcit != prob->data.drone_classes.end(); ++dcit) {
            auto g = prob->graphs.at(*dcit);

            int total_iters = row_num*((*dcit)->duration - 2*g->drone_class->speed+1);
            int iters = 0;
            for(auto i = 0; i<row_num; ++i){
                for(auto j = 2*g->drone_class->speed; j<=(*dcit)->duration; ++j){
                    //如果子问题求解时间超过max_time_to_solve_sp，则退出此次迭代中子问题的求解环节
                    auto sp_end = high_resolution_clock::now();
                    auto sp_time = duration_cast<duration<double>>(sp_end - sp_start).count();
                    if (sp_time - prob->params.max_time_to_solve_sp > prob->params.ppm) {
                        break;
                    }

                    iters++;
                    auto end_j = GraphGenerator::cal_t(prob->data, i, j, false);
                    // std::cerr<<"fffffffffffffffffffffff"<<std::endl;
                    auto sol = solve_by_begin_end_point(g, g->depot_nodes[i], g->depot_nodes[end_j]);
                    // std::cerr<<"fffffffffffffffffffffff"<<std::endl;
                    
                    if (sol.state != 0 && GraphGenerator::lessThanReal(sol.obj_value, 0, prob->params.ppm)){
                        //将sol转化成Solution，然后加入到valid_sols
                        Path new_path;
                        for(int a = 0; a<g->arc_vec.size(); ++a){
                            if (sol.arc_variables[a] == 1) new_path.push_back(g->arc_vec[a]->id);
                        }
                        Solution new_solution = Solution(new_path, sol.obj_value, g);
                        // std::cerr<<"("<<g->depot_nodes[i]->t<<", "<<g->depot_nodes[i]->loc<<")->("<<g->depot_nodes[end_j]->t<<", "<<g->depot_nodes[end_j]->loc<<")"<<std::endl;
                        // new_solution.print();
                        valid_sols.push_back(new_solution);
                    }
                    std::cerr << std::unitbuf << "SP time one execute time: " << sp_time << " seconds" << std::endl;
                }
            }

            if(iters < total_iters){
                std::cerr<<"iters = "<<iters<<"; "<<"total iters = "<<total_iters<<std::endl;
                is_exact = false;
                break;
            }
        }

        if(valid_sols.size() > 0) {
            for(const auto &s : valid_sols) {
                Column col(prob, s, false);
                node_pool.push_back(col);
                global_pool->push_back(col);
            }
            return {(int)valid_sols.size(), is_exact};
        }

        return {0, is_exact};
    }

    SPIntegerSolution SPSolver::solve_by_begin_end_point(std::shared_ptr<const Graph> g, const std::shared_ptr<Node> &begin_node, const std::shared_ptr<Node> &end_node, bool model_dump) const{
        
        IloEnv env;
        IloModel model(env);

        auto row_num = prob->data.num_times;
        auto col_num = prob->data.num_targets+1;
        auto arc_size = g->arc_vec.size();

        IloNumVarArray arc_var(env, arc_size, 0, 1, ILOINT);
        for(int i = 0; i<arc_size; ++i){
            arc_var[i] = IloNumVar(env, 0, 1, ILOINT, ("a_" + std::to_string(i)).c_str());
        }

        IloNumVarArray2 inspection_var(env, row_num);
        for (int i = 0; i < row_num; ++i) {
            auto ans_vars = IloNumVarArray(env, col_num);
            for(int j = 0; j<col_num; ++j){
                ans_vars[j] = IloNumVar(env, 0, 1, IloNumVar::Bool, ("y_" + std::to_string(i)+"_"+std::to_string(j)).c_str());
            }
            inspection_var[i] = ans_vars;
            model.add(inspection_var[i]);
        }

        // std::cerr<<"ffffffggggggggggggggfffffffff"<<std::endl;
        /**剔除一定不相关的点和边**/
        std::vector<TimePoint> unrelated_timepoints = {begin_node->t};
        auto t = end_node->t;
        while(t != begin_node->t){
            unrelated_timepoints.push_back(t);
            t = t+1;
            t = t>=row_num?t-row_num:t;
        }
        // std::cerr<<"ffffffgggggggddddddddddddgggggggfffffffff"<<std::endl;
        
        std::vector<int> unrelated_arcs = {};
        for(auto t: unrelated_timepoints){
            for(int j = 0; j<col_num; ++j){
                if (g->node_matrix[t][j] != begin_node){  //点的右邻边
                    for(auto arc_id: g->node_matrix[t][j]->right_arcs){
                        if (std::find(unrelated_arcs.begin(), unrelated_arcs.end(), arc_id) == unrelated_arcs.end()){
                            unrelated_arcs.push_back(arc_id);
                        }
                    }
                }
                if (g->node_matrix[t][j] != end_node){    //点的左邻边
                    for(auto arc_id: g->node_matrix[t][j]->left_arcs){
                        if (std::find(unrelated_arcs.begin(), unrelated_arcs.end(), arc_id) == unrelated_arcs.end()){
                            unrelated_arcs.push_back(arc_id);
                        }
                    }
                }
            }
        }
        for(auto arc_id: unrelated_arcs){
            model.add(arc_var[arc_id] == 0);
        }
        // std::cerr<<"ffffffgggggggdaaaaaaaaaaaaaaaagggggggfffffffff"<<std::endl;

        /*****两类决策变量之间的联系*****/
        for(auto i = 0; i<row_num; ++i){
            for(auto j = 0; j<col_num; ++j){
                IloExpr expr(env);
                for(auto a = 0; a<arc_size; ++a){
                    expr += arc_var[a]*g->arc_vec[a]->inspections[i][j];
                }
                model.add(expr>=inspection_var[i][j]);
            }
        }
        // std::cerr<<"ffffffggggggqqqqqqqqqqqqqqqqqqggggggggfffffffff"<<std::endl;

        /*****起终点约束*****/
        for(auto node: g->depot_nodes){
            // std::cout<<"Node: ("<<node->id<<", "<<node->t<<", "<<node->loc<<")"<<std::endl;
            // for(auto arc_id: node->left_arcs){ 
            //     std::cout<<arc_id<<"\t";
            // }
            // std::cout<<std::endl;

            // for(auto arc_id: node->right_arcs){ 
            //     std::cout<<arc_id<<"\t";
            // }
            // std::cout<<std::endl;

            IloExpr expr1(env), expr2(env);
            for(auto arc_id: node->right_arcs){
                expr1 += arc_var[arc_id];
            }

            if (node == begin_node){
                model.add(expr1==1);
            }else{
                model.add(expr1==0);
            }

            for(auto arc_id: node->left_arcs){
                expr2 += arc_var[arc_id];
            }

            if (node == end_node){
                model.add(expr2==1);
            }else{
                model.add(expr2==0);
            }
        }
        
        for(auto i = 0; i<row_num; ++i){
            for(auto j = 1; j<col_num; ++j){
                IloExpr expr3(env), expr4(env);
                // if ((i==0 && j==86)) std::cout<<"hhhhh"<<std::endl;
                for(auto arc_id: g->node_matrix[i][j]->left_arcs){ 
                    // if ((i==0 && j==86)){   // || (i==1 && j ==57)
                    //     std::cout<<arc_id<<"\t";
                    // }
                    if (g->arc_vec[arc_id]->left_node->loc != 0 || g->arc_vec[arc_id]->left_node == begin_node 
                        || g->arc_vec[arc_id]->left_node == end_node){
                        expr3 += arc_var[arc_id];
                    }
                }
                // if ((i==0 && j==86)) std::cout<<std::endl;

                for(auto arc_id: g->node_matrix[i][j]->right_arcs){
                    // if ((i==0 && j==86)){   // || (i==1 && j ==57)
                    //     std::cout<<arc_id<<"\t";
                    // }
                    if (g->arc_vec[arc_id]->right_node->loc != 0 || g->arc_vec[arc_id]->right_node == begin_node 
                        || g->arc_vec[arc_id]->right_node == end_node){
                        expr4 += arc_var[arc_id];
                    }
                }
                // if ((i==0 && j==86)) std::cout<<std::endl;

                model.add(expr3 == expr4);  //流平衡约束
            }
        }
        // std::cerr<<"ffffffgggggggggggeeeeeeeeeeeeegggfffffffff"<<std::endl;

        IloExpr expr5(env);
        for(auto i = 0; i<row_num; ++i){
            for(auto j = 1; j<col_num; ++j){
                expr5 += g->graph_dual_properties.inspection_duals[i][j]*inspection_var[i][j];
            }
        }
        for(auto i = 0; i<arc_size; ++i){
            for(auto t = 0; t<row_num; ++t){
                expr5 += arc_var[i]*g->graph_dual_properties.timepoint_duals[t]*g->arc_vec[i]->timepoints[t];
            }
        }
        model.add(IloMinimize(env, expr5));
        // std::cerr<<"ffffffggggggggggyyyyyyyyyyyyyyyyyyyggggfffffffff"<<std::endl;

        IloCplex cplex(model);
        if(model_dump) {
            std::string output_name = "subproblem_problem_"+ std::to_string(begin_node->id)+"_"+std::to_string(end_node->id)+".lp";
            try {
                cplex.exportModel(output_name.c_str());
            } catch(IloException& e) {
                std::cerr << "Export IloException: " << e << std::endl;
            }
        }

        // cplex.setParam(IloCplex::Threads, prob->params.cplex_cores);
        cplex.setOut(env.getNullStream());

        auto solved = false;

        try {
            solved = cplex.solve();
        } catch(IloException &e) {
            std::cerr << "Subproblem Solve IloException: " << e << std::endl;
            throw;
        }

        if(!solved) {
            std::cerr << "Infeasible problem!" << std::endl;
            throw std::runtime_error("Infeasible problem!");
        }
        // std::cerr<<"ffffffggggggggggyyyyyyyyyywwwwwwwwwwwyyyyyyyyyggggfffffffff"<<std::endl;
        
        /******************** get the result of cplex solving ***********************************/
        // std::cerr<<"get the result of cplex solving"<<std::endl;
        auto obj_value = cplex.getObjValue();
        IloNumArray values(env);

        cplex.getValues(values, arc_var);
        // std::cerr<<"ffffffggggggggggyyyyyyyyyywwwwwwwwwwwyyyyyyyyyggggfffffffff"<<std::endl;

        // std::cerr<<"out arc_variables:"<<std::endl;
        auto arc_variables = std::vector<int>();
        for(auto i = 0; i < values.getSize(); i++) {
            auto ans_value = GraphGenerator::equalToReal(values[i], 0, prob->params.ppm)?0:1;
            arc_variables.push_back(ans_value);
            // if (values[i]!=0) std::cerr<<values[i]<<","<<g->arc_vec[i]->id<<"\t";
            // if (values[i]==1) std::cerr<<values[i]<<","<<g->arc_vec[i]->id<<"\t";
            // std::cerr<<i<<": "<<values[i]<<std::endl;
        }
        // std::cerr<<std::endl;
        // std::cerr<<"ffffffggggggggggyyyyyyyyyyyybbbbbbbbbbbbbbyyyyyyyggggfffffffff"<<std::endl;

        // std::cerr<<"out inspection_variables:"<<std::endl;
        auto inspection_variables = std::vector<std::vector<int>>(row_num, std::vector<int>(col_num));
        for(auto i = 0; i < row_num; i++) {
            cplex.getValues(values, inspection_var[i]);
            for(auto j = 0; j < col_num; ++j){
                auto ans_value = GraphGenerator::equalToReal(values[j], 0, prob->params.ppm)?0:1;
                inspection_variables[i][j] = ans_value;
                // std::cerr<<i<<","<<j<<": "<<values[j]<<std::endl;
            }
        }
        // std::cerr<<"ffffffggggggggggyyyyyyyyyyyyyyyjjjjjjjjjjjjjjyyyyggggfffffffff"<<std::endl;

        values.end();
        env.end();

        return SPIntegerSolution(obj_value, inspection_variables, arc_variables);
    }
}
