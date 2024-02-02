
#ifndef NODE_H
#define NODE_H

#include <iostream>
#include <memory>
#include <vector>

#include "drone_class.h"
#include "inspection.h"
#include "target.h"

namespace drone_cover {
    struct Node;
    struct Inspection;
    using Node_Matrix = std::vector<std::vector<std::shared_ptr<Node>>>;
    using Node_Vec = std::vector<std::shared_ptr<Node>>;
    
    struct Node {
        int id;
        int loc;
        int t;

        /**
         * Drone class of the graph where the node resides.
         */
        std::shared_ptr<DroneClass> drone_class;

        /**
         * Adjacent targets within the coverage range of drones
        */
        std::vector<std::shared_ptr<Target>> adjacent_covered_targets;

        /**
         * 左邻边的结点
         */
        Node_Vec left_nodes;

        /**
         * 右邻边的结点
         */
        Node_Vec right_nodes;

        /**
         * 覆盖的inspection
         */
        Inspection_Vec covered_inspections;

        /**
         * 左邻边id集合
         */
        std::vector<int> left_arcs;

        /**
         * 右邻边id集合
         */
        std::vector<int> right_arcs;

        Node() {}

        Node(int id, int t, int loc, std::shared_ptr<DroneClass> drone_class) :
            id(id), loc(loc), t(t), drone_class(drone_class) {
            }

        /**
         * Two nodes are equal if they have the same id.
         * @param other Other node
         * @return      True iff the current and the other nodes are equal
         */
        bool operator==(const Node &other) const;

        bool operator!=(const Node &other) const { return !(*this == other); }
    };

    /**
     * Shortly prints info about a node.
     */
    std::ostream &operator<<(std::ostream &out, const Node &n);
}
#endif
