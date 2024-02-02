
#ifndef SP_SOLVER_H
#define SP_SOLVER_H

#include <utility>

#include "../base/problem.h"
#include "../column/column_pool.h"
#include "sp_integer_solution.h"
#include "../preprocessing/graph_generator.h"
#include <ilcplex/ilocplex.h>

namespace drone_cover {
    using IloNumVarArray2 = IloArray<IloNumVarArray>;
    class SPSolver {
    public:
        std::shared_ptr<const Problem> prob;

        SPSolver(std::shared_ptr<const Problem> prob) : prob(prob){}

        /* Returns how many columns were added to the column pool*/
        std::pair<int,bool> solve(ColumnPool &node_pool, std::shared_ptr<ColumnPool> global_pool, double &time_spent_by_exact_solver) const;

    private:
        bool solution_in_pool(const Solution &s, const ColumnPool &pool) const;
        SPIntegerSolution solve_by_begin_end_point(std::shared_ptr<const Graph> g, const std::shared_ptr<Node> &begin_node, const std::shared_ptr<Node> &end_node, bool model_dump=false) const;
        void print_report(int sols_found, int discarded_prc, int discarded_infeasible, int discarded_generated, int discarded_in_pool, std::ostream &out = std::cerr) const;
    };
}

#endif
