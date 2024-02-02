
#ifndef INSPECTION_H
#define INSPECTION_H

#include <vector>
#include <memory>
#include "node.h"
#include "arc.h"

namespace drone_cover{
    struct Node;

    struct Inspection
    {
        int id;
        int loc;
        int t;
        int profit;

        std::vector<std::shared_ptr<drone_cover::Node>> iscovered_nodes; //被哪些node覆盖，废弃
        std::vector<int> iscovered_arcs; //被哪些arc覆盖，废弃

        Inspection(int id, int t, int loc, int profit):id(id), t(t), loc(loc), profit(profit){}

        bool operator==(const Inspection& other) const{
            if ((id == other.id) ||(loc == other.loc && t == other.t)){
                return true;
            }
            return false;
        }
    };

    using Inspection_Matrix = std::vector<std::vector<std::shared_ptr<Inspection>>>;
    using Inspection_Vec = std::vector<std::shared_ptr<Inspection>>;
}


#endif