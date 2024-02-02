
#include <algorithm>
#include <stdexcept>
#include <unordered_map>
#include <string>
#include <utility>

#include "graph_generator.h"

namespace drone_cover {
    namespace GraphGenerator {
        std::shared_ptr<Graph>
        create_graph(const ProblemData &data, std::shared_ptr<DroneClass> drone_class, Inspection_Matrix inspection_matrix) {
            auto g = std::make_shared<Graph>();
            g->drone_class = drone_class;
            
            auto row_num = data.num_times;
            auto col_num = data.num_targets+1;

            auto node_matrix = Node_Matrix(data.num_times, Node_Vec(data.num_targets+1));
            auto node_vec = Node_Vec(data.num_times);
            auto arc_vec = Arc_Vec();

            /**
             * 创建时空结点矩阵，同时获得无人机在该结点能够覆盖的所有相邻targets集合
             */
            int node_id = 0;
            for(int j = 0; j<col_num; ++j){
                std::vector<std::shared_ptr<Target>> all_adjacent_targets;
                get_all_covered_targets(data.targets[j], all_adjacent_targets, drone_class, 0);
                for(int i = 0; i<row_num; ++i){
                    auto ansnode = std::make_shared<Node>(node_id++, i, j, drone_class);
                    ansnode->adjacent_covered_targets = all_adjacent_targets;
                    if (j==0) { //如果是在depot，则不做覆盖的操作
                        ansnode->adjacent_covered_targets = std::vector<std::shared_ptr<Target>>{};
                        node_vec[i] = ansnode;    //depot点不能覆盖相邻点
                    }
                    node_matrix[i][j] = ansnode;
                }
            }
            g->node_matrix = node_matrix;
            g->depot_nodes = node_vec;

            /**
             * obtain adjacent nodes, arcs and inspections of the node
             */
            // int arc_id = 0;
            for(int i = 0; i<row_num; ++i){
                for(int j = 0; j<col_num; ++j){
                    
                    for(auto target: data.targets[j]->adjacent_targets){
                        /**
                         * obtain adjacent nodes of the node
                         */
                        int move_time = (target->id == j)?1:drone_class->speed; //如果还是同一个target，则只移动一个单位时间
                        int pre_t = cal_t(data, i, move_time,  true);
                        int successor_t = cal_t(data, i, move_time, false);

                        node_matrix[i][j]->left_nodes.push_back(node_matrix[pre_t][target->id]);
                        node_matrix[i][j]->right_nodes.push_back(node_matrix[successor_t][target->id]);

                        /**
                         * create all the arcs
                         */
                        auto arc_id = (int)arc_vec.size();
                        auto ansarc = std::make_shared<Arc>(arc_id, node_matrix[i][j], node_matrix[successor_t][target->id]);
                        auto it = find_arc(arc_vec, ansarc);
                        // auto it = std::find(arc_vec.begin(), arc_vec.end(), ansarc);
                        if (it == -1) {
                            arc_vec.push_back(ansarc);
                        }
                    }

                    /**
                     * obtain inspections of the node
                     */
                    for(auto target: node_matrix[i][j]->adjacent_covered_targets){
                        for(int k = 0; k<std::min(target->revisit_time, data.num_times); ++k){
                            int mt = cal_t(data, i, k, true);
                            node_matrix[i][j]->covered_inspections.push_back(inspection_matrix[mt][target->id]);
                            // inspection_matrix[mt][target->id]->iscovered_nodes.push_back(node_matrix[i][j]);
                        }
                    }
                }
            }
            
            for(auto arc: arc_vec){
                arc->inspections = calculate_arc_inspections(*arc, data);
                arc->timepoints = calculate_arc_timepoints(*arc, data);
            }
            g->arc_vec = arc_vec;

            // std::cerr<<"Arc id is 569 = "<<arc_vec[569]->id<<"\t"<<arc_vec[569]->left_node->t<<"\t"<<arc_vec[569]->left_node->loc<<"\t"<<arc_vec[569]->right_node->t<<"\t"<<arc_vec[569]->right_node->loc<<std::endl;
            // std::cerr<<"Arc id is 568 = "<<arc_vec[568]->id<<"\t"<<arc_vec[568]->left_node->t<<"\t"<<arc_vec[568]->left_node->loc<<"\t"<<arc_vec[568]->right_node->t<<"\t"<<arc_vec[568]->right_node->loc<<std::endl;
            if (arc_vec.size() != (unsigned int)(arc_vec.back()->id+1)){
                std::cerr<<"Arc Vec size = "<<arc_vec.size()<<"\tLast Arc Id = "<<arc_vec.back()->id<<std::endl;
                throw std::runtime_error("Arc id gets wrong!");
            }

            // /**
            //  * 获得覆盖一个inspection的所有arcs集合
            //  */
            // for(auto arc: arc_vec){
            //     connect_arc_with_inspection(arc);
            // }

            /**
             * 获得某个结点所有的arcs集合
             */
            for(int i = 0; i<row_num; ++i){  
                for(int j = 0; j<col_num; ++j){
                    for(auto target: data.targets[j]->adjacent_targets){
                        /**
                         * obtain arcs of the node
                         */
                        int move_time = (target->id == j)?1:drone_class->speed; //如果还是同一个target，则只移动一个单位时间
                        int pre_t = cal_t(data, i, move_time,  true);
                        int successor_t = cal_t(data, i, move_time, false);

                        auto ansrightarc = std::make_shared<Arc>(0, node_matrix[i][j], node_matrix[successor_t][target->id]);
                        auto ansleftarc = std::make_shared<Arc>(0, node_matrix[pre_t][target->id], node_matrix[i][j]);
                        auto right_it = find_arc(arc_vec, ansrightarc);
                        auto left_it = find_arc(arc_vec, ansleftarc);
                        if (right_it != -1) {
                            if (std::find(node_matrix[i][j]->right_arcs.begin(), node_matrix[i][j]->right_arcs.end(), right_it) != node_matrix[i][j]->right_arcs.end()){
                                std::cerr<<i<<","<<j<<": "<<"right arc gets repeated!"<<std::endl;
                                throw std::runtime_error("Arc id gets repeated!");
                            }
                            node_matrix[i][j]->right_arcs.push_back(right_it);
                        }

                        if (left_it != -1) {
                            if (std::find(node_matrix[i][j]->left_arcs.begin(), node_matrix[i][j]->left_arcs.end(), left_it) != node_matrix[i][j]->left_arcs.end()){
                                std::cerr<<i<<","<<j<<": "<<"left arc gets repeated!"<<std::endl;
                                throw std::runtime_error("Arc id gets repeated!");
                            }
                            node_matrix[i][j]->left_arcs.push_back(left_it);
                        }
                    }
                    // for(auto arc_id: node_matrix[i][j]->left_arcs){
                    //     if ((i==1 && j==0)){   // || (i==1 && j ==57)
                    //         std::cerr<<arc_id<<"\t";
                    //     }
                    // }
                    // for(auto arc_id: node_matrix[i][j]->right_arcs){
                    //     if ((i==0 && j==0)){   // || (i==1 && j ==57)
                    //         std::cerr<<arc_id<<"\t";
                    //     }
                    // }
                }
            }
            // throw std::runtime_error("Arc id gets wrong!");

            return g;
        }

        int cal_t(const ProblemData &data, int t, int move_time, bool is_preorder){
            int res_t;
            if (is_preorder){   //如果是前序结点
                res_t = t-move_time;
                res_t = res_t>=0?res_t:res_t+data.num_times;
            }else{
                res_t = t+move_time;
                res_t = res_t>=data.num_times?res_t-data.num_times:res_t;
            }
            return res_t;
        }

        // 递归函数，用于获取一个 Target 相邻的cover range范围内的所有相邻 Targets
        void get_all_covered_targets(const std::shared_ptr<Target>& target, std::vector<std::shared_ptr<Target>>& all_adjacent_targets, std::shared_ptr<DroneClass> drone_class, int layer)
        {
            if (layer >= drone_class->range){
                return ;
            }
            for (const auto& adjacent_target : target->adjacent_targets)
            {
                // 检查是否已经在集合中，避免重复添加
                if (std::find(all_adjacent_targets.begin(), all_adjacent_targets.end(), adjacent_target) == all_adjacent_targets.end())
                {
                    all_adjacent_targets.push_back(adjacent_target);
                    // 递归调用以获取相邻的相邻 Targets
                    get_all_covered_targets(adjacent_target, all_adjacent_targets, drone_class, layer+1);
                }
            }
        }

        // void connect_arc_with_inspection(std::shared_ptr<Arc>& arc){
        //     for(auto inspection: arc->left_node->covered_inspections){
        //         if (std::find(inspection->iscovered_arcs.begin(), inspection->iscovered_arcs.end(), arc->id)==inspection->iscovered_arcs.end()){
        //             inspection->iscovered_arcs.push_back(arc->id);
        //         }
        //     }
        //     for(auto inspection: arc->right_node->covered_inspections){
        //         if (std::find(inspection->iscovered_arcs.begin(), inspection->iscovered_arcs.end(), arc->id)==inspection->iscovered_arcs.end()){
        //             inspection->iscovered_arcs.push_back(arc->id);
        //         }
        //     }
        // }

        /**
         * @brief 在arc集合中查找是否存在arc?如果存在则返回其id，否则返回-1
         * 
         * @param arc_vec 
         * @param arc 
         * @return int 
         */
        int find_arc(Arc_Vec arc_vec, std::shared_ptr<Arc> arc){
            for(int i = 0; i<arc_vec.size(); ++i){
                if (arc_vec[i]->left_node == arc->left_node && arc_vec[i]->right_node == arc->right_node){
                    return arc_vec[i]->id;
                }
            }
            return -1;
        }

        //计算一条arc上所覆盖的inspections
        std::vector<std::vector<int>> calculate_arc_inspections(const Arc& arc, const ProblemData &data){
            auto row_num = data.num_times;
            auto col_num = data.num_targets+1;

            // std::cout<<"Arc nodes: ("<<arc.left_node->t<<","<<arc.left_node->loc<<")--"<<"("<<arc.right_node->t<<","<<arc.right_node->loc<<")"<<std::endl;
            std::vector<std::vector<int>> result = std::vector<std::vector<int>>(row_num, std::vector<int>(col_num, 0));
            for(auto inspection: arc.left_node->covered_inspections){
                // std::cout<< "(t, loc) = "<< inspection->t << ", " << inspection->loc <<std::endl;
                result[inspection->t][inspection->loc] = 1;
            }
            for(auto inspection: arc.right_node->covered_inspections){
                // std::cout<< "(t, loc) = "<< inspection->t << ", " << inspection->loc <<std::endl;
                result[inspection->t][inspection->loc] = 1;
            }
            
            return result;
        }

        //计算一条arc上所飞行和正在飞行的时间点
        std::vector<int> calculate_arc_timepoints(const Arc& arc, const ProblemData &data){
            auto row_num = data.num_times;
            auto result = std::vector<int>(row_num);

            auto t = arc.left_node->t;
            while(t != arc.right_node->t){
                result[t] = 1;
                t = t+1;
                t = t>=row_num?t-row_num:t;
            }
            return result;
        }

        bool lessThanReal(const double &lhs, const double &rhs, const double &threshold) {
            return lhs < rhs - threshold;
        }


        bool greaterThanReal(const double &lhs, const double &rhs, const double &threshold) {
            return lhs > rhs + threshold;
        }


        bool equalToReal(const double &lhs, const double &rhs, const double &threshold) {
            return !(lessThanReal(lhs, rhs, threshold) || greaterThanReal(lhs, rhs, threshold));
        }
    }
}