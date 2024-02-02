
#include <iostream>

#include "problem.h"

namespace drone_cover {
    Problem::Problem(const std::string &params_file, const std::string &data_file) : params{params_file}, data{data_file}, graphs{} {
        inspection_matrix = Inspection_Matrix(data.num_times, Inspection_Vec(data.num_targets+1));
        int inspection_id = 0;
        for(int i = 0; i<data.num_times; ++i){
            for(int j = 0; j<data.num_targets+1; ++j){
                inspection_matrix[i][j] = std::make_shared<Inspection>(inspection_id++, i, j, data.targets[j]->profit);
            }
        }
        
        for(auto drone_class : data.drone_classes) {
            auto g = GraphGenerator::create_graph(data, drone_class, inspection_matrix);

            std::cerr << "Graph for " << drone_class->id<<std::endl;

            graphs.emplace(drone_class, g);
        }
    }
}
