
#ifndef COLUMN_H
#define COLUMN_H

#include <memory>
#include <string>
#include <vector>

#include "../base/problem.h"
#include "solution.h"

namespace drone_cover {

    struct Column {
        std::shared_ptr<const Problem> prob;
        Solution sol;
        double reduced_cost;
        bool dummy;

        Column(std::shared_ptr<const Problem> prob) : prob(prob) {}

        Column(std::shared_ptr<const Problem> prob, const Solution &sol): prob(prob), sol(sol), reduced_cost(-sol.reduced_cost), dummy(false){}

        Column(std::shared_ptr<const Problem> prob, const Solution &sol, bool dummy) :
               prob(prob), sol(sol), dummy(dummy), reduced_cost(-sol.reduced_cost){}

        void make_dummy();
    };

    std::ostream &operator<<(std::ostream &out, const Column &c);
}

#endif