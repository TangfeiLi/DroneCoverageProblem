
#include <stdexcept>
#include <string>
#include <vector>

#include "mp_solver.h"

namespace drone_cover {
    MPSolver::IloData MPSolver::solve(const ColumnPool &pool, bool linear,  bool model_dump) const {

        IloEnv env;
        IloModel model(env);

        auto row_num = prob->data.num_times;
        auto col_num = prob->data.num_targets+1;
        auto drone_class_num = prob->data.num_drone_classes;
        std::stringstream cst_name;

        IloNumVarArray route_var(env);
        IloNumVarArray2 inspection_var(env, row_num);
        for (int i = 0; i < row_num; ++i) {
            auto ans_vars = IloNumVarArray(env, col_num);
            for(int j = 0; j<col_num; ++j){
                ans_vars[j] = IloNumVar(env, 0, 1, (linear ? IloNumVar::Float : IloNumVar::Bool), ("y_" + std::to_string(i)+"_"+std::to_string(j)).c_str());
            }
            inspection_var[i] = ans_vars;
            model.add(inspection_var[i]);
        }
        IloRangeArray inspection_constr(env);
        IloRangeArray timepoint_constr(env);

        IloExpr obj_expr(env);
        for(auto i = 0; i<row_num; ++i){
            for(auto j = 0; j<col_num; ++j){
                try{
                    obj_expr += inspection_var[i][j]*prob->inspection_matrix[i][j]->profit;
                } catch(IloException& e) {
                    std::cerr << "Objective IloException: " << e << std::endl;
                    throw;
                }
            }
        }
        IloObjective obj = IloMaximize(env, obj_expr);

        // Inspection constraints
        for(auto i = 0; i<row_num; ++i){
            for(auto j = 0; j<col_num; ++j){
                try{
                    cst_name << "inspection_" << i << "_" << j;
                    inspection_constr.add(IloRange(env, 0, -inspection_var[i][j], IloInfinity, cst_name.str().c_str()));
                    cst_name.str("");
                } catch(IloException& e) {
                    std::cerr << "InspectionRow IloException: " << e << std::endl;
                    throw;
                }
            }
        }

        // drone constraints
        for(auto d = 0; d < drone_class_num; ++d){
            for(auto i = 0; i < row_num; i++) {
                try {
                    cst_name << "timepoint_" << i;
                    timepoint_constr.add(IloRange(env, -IloInfinity, prob->data.drone_classes[d]->num_drones, cst_name.str().c_str()));
                    cst_name.str("");
                }catch(IloException& e) {
                    std::cerr << "TimePointRow IloException: " << e << std::endl;
                    throw;
                }
            }
        }

        auto col_n = 0;
        for(auto cit = pool.begin(); cit != pool.end(); ++cit) {
            // auto cost = cit->dummy?prob->params.dummy_column_price:0;
            IloNumColumn ilo_c = obj(0);

            for(auto i = 0; i<row_num; ++i){
                for(auto j = 0; j<col_num; ++j){
                    try{
                        ilo_c += inspection_constr[i*col_num+j](cit->sol.inspections[i][j]);
                    } catch(IloException& e) {
                        std::cerr << "Cols InspectionRow IloException: " << e << std::endl;
                        throw;
                    }
                }
            }

            for(auto i = 0; i<row_num; ++i){
                try {
                    ilo_c += timepoint_constr[cit->sol.drone_class->id*row_num+i](cit->sol.used_timepoints[i]);
                } catch(IloException& e) {
                    std::cerr << "Cols TimePoint IloExceptions: " << e << std::endl;
                    throw;
                }
            }

            // IloNumVar v(ilo_c, 0, IloInfinity, (linear ? IloNumVar::Float : IloNumVar::Bool), ("theta_" + std::to_string(col_n++)).c_str());
            IloNumVar v(ilo_c, 0, 1, (linear ? IloNumVar::Float : IloNumVar::Bool), ("x_" + std::to_string(col_n++)).c_str());
            route_var.add(v);
        }

        model.add(obj);
        model.add(inspection_constr);
        model.add(timepoint_constr);

        IloCplex cplex(model);

        if(model_dump) {
            std::string output_name =  + "master_problem_xx.lp";
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
            std::cerr << "Solve IloException: " << e << std::endl;
            throw;
        }


        if(!solved) {
            throw std::runtime_error("Infeasible problem!");
        }

        return std::make_tuple(env, route_var, inspection_var, inspection_constr, timepoint_constr, cplex);
    }

    MPLinearSolution MPSolver::solve_lp(const ColumnPool &pool, bool model_dump) const {
        IloEnv env;
        IloNumVarArray route_var(env);
        IloNumVarArray2 inspection_var(env);

        IloRangeArray inspection_constr;
        IloRangeArray timepoint_constr;
        IloCplex cplex;

        auto row_num = prob->data.num_times;
        auto col_num = prob->data.num_targets+1;    //包含depot
        auto drone_class_num = prob->data.num_drone_classes; //虽然是为多类无人机设计的，但是当前设定只有1类无人机

        std::tie(env, route_var, inspection_var, inspection_constr, timepoint_constr, cplex) = solve(pool, true, model_dump);

        double obj_value = cplex.getObjValue();

        IloNumArray values(env);

        cplex.getDuals(values, inspection_constr);

        // std::cerr<<"out inspection_dual:"<<std::endl;
        auto inspection_duals = std::vector<std::vector<double>>(row_num, std::vector<double>(col_num));
        for(auto i = 0; i<row_num; ++i){
            for(auto j = 0; j<col_num; ++j){
                inspection_duals[i][j] = values[i*col_num+j];
                // std::cerr<<i<<","<<j<<": "<< inspection_duals[i][j]<<std::endl;
            }
        }

        cplex.getDuals(values, timepoint_constr);
        // std::cerr<<"out timepoint_dual:"<<std::endl;
        auto timepoint_duals = std::vector<std::vector<double>>(drone_class_num, std::vector<double>(row_num));
        for(auto d = 0; d < drone_class_num; ++d) {
            for(auto i = 0; i < row_num; ++i){
                timepoint_duals[d][i] = values[d*col_num+i];
                // std::cerr<<d<<","<<i<<": "<< timepoint_duals[d][i] <<std::endl;
            }
        }

        cplex.getValues(values, route_var);

        // std::cerr<<"out route_value:"<<std::endl;
        auto route_variables = std::vector<double>();
        for(auto i = 0; i < values.getSize(); i++) {
            route_variables.push_back(values[i]);
            // std::cerr<<i<<": "<<values[i]<<std::endl;
        }

        // std::cerr<<"out inspection_value:"<<std::endl;
        auto inspection_variables = std::vector<std::vector<double>>(row_num, std::vector<double>(col_num));
        for(auto i = 0; i < row_num; i++) {
            cplex.getValues(values, inspection_var[i]);
            for(auto j = 0; j < col_num; ++j){
                inspection_variables[i][j] = values[j];
                // std::cerr<<i <<"," <<j<<": "<<values[i]<<std::endl;
            }
        }

        /**reduced cost**/
        cplex.getReducedCosts(values, route_var);
        auto route_reduced_costs = std::vector<double>();
        for(auto i = 0; i < values.getSize(); i++) {
            route_reduced_costs.push_back(values[i]);
            // std::cerr<<i<<": "<<values[i]<<std::endl;
        }

        values.end();
        env.end();

        return MPLinearSolution(obj_value, route_variables, inspection_variables, timepoint_duals, inspection_duals, route_reduced_costs);
    }

    MPIntegerSolution MPSolver::solve_mip(const ColumnPool &pool, bool model_dump) const {
        IloEnv env;
        IloNumVarArray route_var(env);
        IloNumVarArray2 inspection_var(env);

        IloRangeArray inspection_constr;
        IloRangeArray timepoint_constr;
        IloCplex cplex;

        auto row_num = prob->data.num_times;
        auto col_num = prob->data.num_targets+1;    //包含depot
        auto drone_class_num = prob->data.num_drone_classes; //虽然是为多类无人机设计的，但是当前设定只有1类无人机

        std::tie(env, route_var, inspection_var, inspection_constr, timepoint_constr, cplex) = solve(pool, false, model_dump);

        double obj_value = cplex.getObjValue();

        IloNumArray values(env);

        cplex.getValues(values, route_var);

        // std::cout<<"out route_value:"<<std::endl;
        auto route_variables = std::vector<int>();
        for(auto i = 0; i < values.getSize(); i++) {
            auto ans_value = GraphGenerator::equalToReal(values[i], 0, prob->params.ppm)?0:1;
            route_variables.push_back(ans_value);
            // std::cout<<i<<": "<<values[i]<<std::endl;
        }

        // std::cout<<"out inspection_value:"<<std::endl;
        auto inspection_variables = std::vector<std::vector<int>>(row_num, std::vector<int>(col_num));
        for(auto i = 0; i < row_num; i++) {
            cplex.getValues(values, inspection_var[i]);
            for(auto j = 0; j < col_num; ++j){
                auto ans_value = GraphGenerator::equalToReal(values[j], 0, prob->params.ppm)?0:1;
                inspection_variables[i][j] = ans_value;
                // std::cout<<i <<"," <<j<<": "<<values[i]<<std::endl;
            }
        }

        values.end();
        env.end();

        return MPIntegerSolution(obj_value, route_variables, inspection_variables);
    }
}