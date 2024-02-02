
#include <stdexcept>
#include <iostream>
#include <istream>
#include <fstream>
#include <sstream>

#include "problem_data.h"

namespace drone_cover {
    ProblemData::ProblemData(const std::string &data_file_name) {
        std::ifstream ins(data_file_name);
        if (!ins) throw std::exception("Input file error.");

        std::string temp;

        std::getline(ins, temp);
        ins >> num_targets >> num_drone_classes >> num_times;

        std::getline(ins, temp);
        std::getline(ins, temp);

        for(int i = 0; i<num_drone_classes;++i){
            std::getline(ins, temp);
            std::istringstream iss(temp);
            int id; int num_drones; int duration; int speed; int range;
            iss >> id >> num_drones >> duration >> speed >> range;
            // std::cerr<< id <<num_drones << duration << speed << range <<std::endl;
            auto ansdrone = std::make_shared<DroneClass>(id, range, num_drones, duration, speed);
            // ansdrone->print();
            drone_classes.push_back(ansdrone);
        }

        std::getline(ins, temp);
        std::vector<std::vector<int>> adjacency_matrix(num_targets+1);
        for(int i = 0; i<num_targets+1; ++i) {
            std::getline(ins, temp);
            std::istringstream iss(temp);
            std::vector<int> row;

            int value;
            iss>>value;
            while (iss >> value) {
                row.push_back(value);
            }

            adjacency_matrix[i] = row;
        }

        std::getline(ins, temp);
        for(int i = 0; i<num_targets+1; ++i){
            int id, revisit_time, profit;
            ins>>id>>revisit_time>>profit;
            auto anstarget = std::make_shared<Target>(id,revisit_time, profit);
            targets.push_back(anstarget);
        }
        std::cout<<"the size of targets = "<<targets.size()<<std::endl;

        for(int i = 0; i<num_targets+1; ++i){
            for(auto j: adjacency_matrix[i]){
                targets[i]->adjacent_targets.push_back(targets[j]);
            }
        }
        
        // print();
    }

    void ProblemData::print(){
        std::cout<<"****************ProblemData*******************"<<std::endl;
        std::cout<<"num_times: "<< num_times <<std::endl;
        std::cout<<"num_drone_classes: "<< num_drone_classes <<std::endl;
        std::cout<<"num_targets: "<< num_targets <<std::endl;
        std::cout<<"****************ProblemData*******************"<<std::endl;
    }
}