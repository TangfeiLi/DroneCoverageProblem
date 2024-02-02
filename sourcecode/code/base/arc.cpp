
#include "arc.h"

namespace drone_cover {
    void Arc::print(){
        std::cout<<"Arc ID: "<<id<<std::endl;
        std::cout<<"("<<left_node->t<<","<<left_node->loc<<")-("<<right_node->t<<","<<right_node->loc<<")"<<std::endl;

        std::cout<<"Inspections: "<<std::endl;
        for(auto i = 0; i<inspections.size(); ++i){
            for(auto j = 0; j<inspections[0].size(); ++j){
                std::cout<<inspections[i][j]<<"\t";
            }
            std::cout<<std::endl;
        }

        std::cout<<"Used_timepoints: "<<std::endl;
        for(auto i = 0; i<timepoints.size(); ++i){
            std::cout<<timepoints[i]<<"\t";
        }
        std::cout<<std::endl;
    }
}