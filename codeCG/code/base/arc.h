
#ifndef ARC_H
#define ARC_H

#include <iostream>
#include <vector>
#include <memory>
#include "node.h"

namespace drone_cover {
    struct Node;
    
    struct Arc {
        int id;

        /**
         * 左结点
         */
        std::shared_ptr<Node> left_node;

        /**
         * 右结点
         */
        std::shared_ptr<Node> right_node;

        std::vector<std::vector<int>> inspections; //如果对应位置的inspection被 Arc cover，则为1; 否则为0
        std::vector<int> timepoints; //如果arc在此刻起飞或者飞行 ，则为1; 否则为0

        Arc(){}

        Arc(int id, std::shared_ptr<Node> left_node, std::shared_ptr<Node> right_node) :
            id(id), left_node(left_node), right_node(right_node) {}
        
        // Arc(std::shared_ptr<Node> left_node, std::shared_ptr<Node> right_node) :
        // left_node(left_node), right_node(right_node) {}

        bool operator==(const Arc& other) const {
            if (left_node && other.left_node && right_node && other.right_node){
                return left_node == other.left_node && right_node == other.right_node;
            }
            return false;
        }

        void print();
    };

    using Arc_Vec = std::vector<std::shared_ptr<Arc>>;
    using Arc_id = int;
}

#endif
