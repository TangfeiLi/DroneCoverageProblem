
#ifndef TARGET_H
#define TARGET_H

#include <vector>
#include <memory>

namespace drone_cover{
    
    struct Target
    {
        int id;
        int revisit_time;
        int profit;
        std::vector<std::shared_ptr<Target>> adjacent_targets;

        Target(int id, int revisit_time, int profit):id(id), profit(profit), revisit_time(revisit_time){}
    };
    
}


#endif