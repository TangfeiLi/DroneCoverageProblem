
#ifndef MP_LINEAR_SOLUTION
#define MP_LINEAR_SOLUTION

#include <vector>

namespace drone_cover {
    struct MPLinearSolution {
        double obj_value;
        std::vector<double> route_var;
        std::vector<std::vector<double>> inspection_var;

        std::vector<std::vector<double>> timepoint_duals;
        std::vector<std::vector<double>> inspection_duals;

        std::vector<double> route_reduced_costs;

        MPLinearSolution(double obj_value, const std::vector<double> &route_var, const std::vector<std::vector<double>> &inspection_var, 
                        const std::vector<std::vector<double>> &timepoint_duals, const std::vector<std::vector<double>> &inspection_duals,
                        const std::vector<double> &route_reduced_costs) :
                         obj_value(obj_value), route_var(route_var), inspection_var(inspection_var), timepoint_duals(timepoint_duals),
                         inspection_duals(inspection_duals), route_reduced_costs(route_reduced_costs) {}
    };
}

#endif