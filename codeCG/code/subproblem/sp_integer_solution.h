
#ifndef SP_INTEGER_SOLUTION
#define SP_INTEGER_SOLUTION

#include <vector>

namespace drone_cover {
    struct SPIntegerSolution {
        int state; //求解状态: 0无解, 1可行, 2最优解
        double obj_value;
        std::vector<std::vector<int>> inspection_variables;
        std::vector<int> arc_variables;

        SPIntegerSolution(double obj_value, const std::vector<std::vector<int>> &inspection_variables, const std::vector<int> &arc_variables) : 
                    obj_value(obj_value), inspection_variables(inspection_variables),  arc_variables(arc_variables), state(2){}
    };
}

#endif