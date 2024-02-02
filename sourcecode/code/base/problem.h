
#ifndef PROBLEM_H
#define PROBLEM_H

#include <memory>
#include <string>
#include <unordered_map>

#include "graph.h"
#include "drone_class.h"
#include "../preprocessing/problem_data.h"
#include "../preprocessing/program_params.h"
#include "../preprocessing/graph_generator.h"

namespace drone_cover {
    using GraphMap = std::unordered_map<std::shared_ptr<DroneClass>, std::shared_ptr<Graph>>;

    struct Problem {
        /**
         * Algorithm parameters.
         */
        ProgramParams params;

        /**
         * Problem data.
         */
        ProblemData data;

        /**
         * Inspection Matrix
         */
        Inspection_Matrix inspection_matrix;

        /**
         * List of graphs, one for each drone class.
         */
        GraphMap graphs;

        Problem(const std::string &params_file, const std::string &data_file);
    };
}

#endif