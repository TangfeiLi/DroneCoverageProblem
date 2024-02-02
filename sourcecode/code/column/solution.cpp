

#include <limits>
#include <iostream>

#include "solution.h"

namespace drone_cover {
    void Solution::print(std::ostream& outputStream, bool detail) const{
        if (detail){
            outputStream<<"Arc id of the Path:"<<std::endl;
            for(auto id: path){
                outputStream<<id<<"\t";
            }
            outputStream<<std::endl;
            outputStream<<"Reduced cost: "<< reduced_cost <<std::endl;
        }

        outputStream<<"Nodes of the Path: "<<std::endl;
        // outputStream<<"("<<g->arc_vec[path[0]]->left_node->t<<","<<g->arc_vec[path[0]]->left_node->loc<<")->";
        for(auto id: path){
            outputStream<<"("<<g->arc_vec[id]->left_node->t<<","<<g->arc_vec[id]->left_node->loc<<")->";
            outputStream<<"("<<g->arc_vec[id]->right_node->t<<","<<g->arc_vec[id]->right_node->loc<<")->";
        }
        outputStream<<std::endl;

        if (detail){
            for(auto i = 0; i<trajectory_nodes.size(); ++i){
                for(auto j = 0; j<trajectory_nodes[0].size(); ++j){
                    outputStream<<trajectory_nodes[i][j]<<"\t";
                }
                outputStream<<std::endl;
            }

            outputStream<<std::endl<<"Flight time = "<<flight_time<<std::endl;

            outputStream<<"Inspections: "<<std::endl;
            for(auto i = 0; i<inspections.size(); ++i){
                for(auto j = 0; j<inspections[0].size(); ++j){
                    outputStream<<inspections[i][j]<<"\t";
                }
                outputStream<<std::endl;
            }

            outputStream<<"Used_timepoints: "<<std::endl;
            for(auto i = 0; i<used_timepoints.size(); ++i){
                outputStream<<used_timepoints[i]<<"\t";
            }
            outputStream<<std::endl;
        }
    }

}
