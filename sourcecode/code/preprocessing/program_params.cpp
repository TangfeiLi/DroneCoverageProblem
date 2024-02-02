
#include "program_params.h"

namespace drone_cover {
    ProgramParams::ProgramParams(const std::string &data_file_name) {
        std::ifstream f(data_file_name);
        try {
            json json_data = json::parse(f);
            
            json_data.at("max_cols_to_solve_mp").get_to(max_cols_to_solve_mp);
            json_data.at("max_time_to_solve_sp").get_to(max_time_to_solve_sp);
            json_data.at("cplex_cores").get_to(cplex_cores);
            json_data.at("time_limit_in_s").get_to(time_limit_in_s);
            json_data.at("dummy_column_price").get_to(dummy_column_price);
            json_data.at("ppm").get_to(ppm);
        } catch (const nlohmann::json::parse_error& e) {
            std::cerr << "JSON parse error: " << e.what() << " at position " << e.byte << std::endl;
        } catch (const std::exception& e) {
            std::cerr << "Error: " << e.what() << std::endl;
        }

        // print();
    }

    void ProgramParams::print(){
        std::cout<<"****************ProgramParams*******************"<<std::endl;
        std::cout<<"max_cols_to_solve_mp: "<< max_cols_to_solve_mp <<std::endl;
        std::cout<<"max_time_to_solve_sp: "<< max_time_to_solve_sp <<std::endl;
        std::cout<<"cplex_cores: "<< cplex_cores <<std::endl;
        std::cout<<"time_limit_in_s: "<< time_limit_in_s <<std::endl;
        std::cout<<"ppm: "<< ppm <<std::endl;
        std::cout<<"****************ProgramParams*******************"<<std::endl;
    }
}
