
#ifndef GRAPH_H
#define GRAPH_H

#include <iostream>
#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <set>

#include "arc.h"
#include "node.h"
#include "drone_class.h"
#include "graph_properties.h"

namespace drone_cover {
    using Path = std::vector<Arc_id>;
    using TimePoint = int;

    struct Graph {
        /**
         * Drone class associated with the graph.
         */
        std::shared_ptr<DroneClass> drone_class;

        /**
         * Dual values
        */
        GraphProperties graph_dual_properties;

        /**
         * node matrix and depot nodes vector
        */
        Node_Matrix node_matrix;
        Node_Vec depot_nodes;

        /**
         * arc vector
        */
        Arc_Vec arc_vec;

        /**
         * Inspection matrix
        */
        // Inspection_Matrix inspection_matrix;

        Graph() {}

        /**
         * Prints basic information about the graph.
         * @param detailed  If true, will also print the list of vertices and edges
         */
        void print(bool detailed = false) const;

        /**
         * Gives the inspections of a path (as the union of the inspections of its arcs).
         * @param p The path
         * @return  The path's inspections
         */
        std::vector<std::vector<int>> calculate_path_inspections(const Path& p) const;
        std::vector<TimePoint> calculate_path_timepoints(const Path& p) const;
        int calculate_path_flighttime(const Path& p) const;
        std::vector<std::vector<int>> calculate_path_trajectory(const Path& p) const;
    };
}

#endif