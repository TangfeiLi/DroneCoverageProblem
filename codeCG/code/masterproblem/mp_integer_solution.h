
#ifndef MP_INTEGER_SOLUTION
#define MP_INTEGER_SOLUTION

#include <vector>

namespace drone_cover {
    struct MPIntegerSolution {
        double obj_value;
        std::vector<int> route_var;
        std::vector<std::vector<int>> inspection_var;

        // MPIntegerSolution(){}

        MPIntegerSolution(double obj_value, const std::vector<int> &route_var, const std::vector<std::vector<int>> &inspection_var) : 
                        obj_value(obj_value), route_var(route_var), inspection_var(inspection_var) {}
    };
}

#endif