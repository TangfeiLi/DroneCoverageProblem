
#ifndef MP_SOLVER_H
#define MP_SOLVER_H

#include <cstring>
#include <memory>
#include <utility>

#include <ilcplex/ilocplex.h>

#include "../base/problem.h"
#include "../column/column_pool.h"
#include "mp_linear_solution.h"
#include "mp_integer_solution.h"

namespace drone_cover {
    using IloNumVarArray2 = IloArray<IloNumVarArray>;
    struct MPSolver {
        std::shared_ptr<const Problem> prob;

        MPSolver(std::shared_ptr<const Problem> prob) : prob(prob) {}

        MPLinearSolution solve_lp(const ColumnPool &pool, bool model_dump=false) const;
        MPIntegerSolution solve_mip(const ColumnPool &pool, bool model_dump=false) const;

    private:
        using IloData = std::tuple<IloEnv, IloNumVarArray, IloNumVarArray2, IloRangeArray, IloRangeArray, IloCplex>;
        IloData solve(const ColumnPool &pool, bool linear,  bool model_dump) const;
    };
}

#endif