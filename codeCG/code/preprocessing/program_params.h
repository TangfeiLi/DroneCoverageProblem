
#ifndef PROGRAM_PARAMS_H
#define PROGRAM_PARAMS_H

#include <string>
#include <iostream>
#include <fstream>
#include <istream>
#include "../base/json.hpp"

namespace drone_cover {
    using json = nlohmann::json;

    class ProgramParams {
    public:
        int max_cols_to_solve_mp;
        int max_time_to_solve_sp;
        int cplex_cores;
        int time_limit_in_s;
        double ppm; //非常小的常数
        long dummy_column_price;

        ProgramParams(const std::string &data_file_name = "data/program_params.json");

        void print();
    };
}

#endif
