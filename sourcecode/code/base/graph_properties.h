#ifndef GRAPH_PROPERTIES_H
#define GRAPH_PROPERTIES_H

#include <memory>
#include <utility>
#include <vector>

namespace drone_cover {

    struct GraphProperties {
        /**
         * Dual value associated with inspections;
         */
        std::vector<std::vector<double>> inspection_duals;

        /**
         * Dual value associated with the drone class.
         */
        std::vector<double> timepoint_duals;

        GraphProperties(){}

        GraphProperties(const std::vector<std::vector<double>> &inspection_duals, const std::vector<double> &timepoint_duals) :
            inspection_duals(inspection_duals), timepoint_duals(timepoint_duals) {}
        
        void print(){
            std::cerr<<"*****************Print Dual values*******************"<<std::endl;
            std::cerr<<"Inspection duals: "<<std::endl;
            for(auto i = 0; i<inspection_duals.size(); ++i){
                for(auto j = 0; j<inspection_duals[0].size(); ++j){
                    std::cerr<<inspection_duals[i][j]<<"\t";
                }
                std::cerr<<std::endl;
            }

            std::cerr<<"Timepoint duals: "<<std::endl;
            for(auto i = 0; i<timepoint_duals.size(); ++i){
                std::cerr<<timepoint_duals[i]<<"\t";
            }
            std::cerr<<std::endl;
            std::cerr<<"*****************Print Dual values*******************"<<std::endl;
        }
    };
}
#endif