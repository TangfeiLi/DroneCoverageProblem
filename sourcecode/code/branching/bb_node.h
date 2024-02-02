
#ifndef BB_NODE_H
#define BB_NODE_H

#include <memory>
#include <utility>
#include <vector>

#include "../base/problem.h"
#include "../column/column_pool.h"
#include "../masterproblem/mp_solver.h"
#include "../subproblem/sp_solver.h"

namespace drone_cover {
    enum class StateType {
        EXACT, MP_HEURISTIC, SP_HEURISTIC, NONE
    };

    class BBNode {
        static constexpr double EPS = 1e-9;

    public:
        std::shared_ptr<const Problem> prob;

        std::shared_ptr<ColumnPool> pool;   //global pool
        ColumnPool local_pool;

        /*  The integer optimal columns selected by the LP solver with the coefficient (由lp solver求解得到整数解的所有column和对应变量值) */
        std::vector<std::pair<Column, double>> base_columns;
        /*  The integer optimal columns selected by the MIP solver with the coefficient (由mip solver求解得到整数解的所有column和对应变量值)*/
        std::vector<std::pair<Column, double>> mip_base_columns;
        /*  The linear optimal columns selected by the LP solver with the coefficient*/
        std::vector<std::pair<Column, double>> lp_base_columns;

        /*  LP Solution */
        double sol_value;
        /*  MIP Solution */
        double mip_sol_value;
        /*  LB of father node, used to determine the most promising nodes in the queue. The root node does not have any. */
        // boost::optional<double> father_lb;
        double father_lb;
        /*  solving state*/
        StateType state;

        /* Depth in the BB tree */
        int depth;

        /* Node name */
        std::string name;

        /*  Used to determine if a solution is integral, or with cost < 0 */
        static constexpr double cplex_epsilon = 0.000001;

        /* Should we still try to run the ESPPRC labelling at this node? */
        bool try_elementary;

        /* Time spent on SP vs MP (avg and total) */
        std::vector<double> all_times_spent_on_sp;
        double avg_time_spent_on_sp;
        double total_time_spent_on_sp;
        double total_time_spent_on_mp;
        double total_time_spent;
        double max_time_spent_by_exact_solver;

        BBNode() {}

        BBNode(std::shared_ptr<const Problem> prob,
               std::shared_ptr<ColumnPool> pool,
               const ColumnPool &local_pool,
               int depth = 0,
               StateType state = StateType::NONE,
               std::string name = "root",
               bool try_elementary = true,
               double avg_time_spent_on_sp = 0,
               double total_time_spent_on_sp = 0,
               double total_time_spent_on_mp = 0,
               double total_time_spent = 0,
               double max_time_spent_by_exact_solver = 0);

        BBNode(const BBNode& father, std::string name);

        void solve(unsigned int node_number);

        bool solve_integer(const ColumnPool &feasible_columns);

        void check_column_reduced_cost(const Column &column, const GraphProperties &graph_dual_properties);

        // bool is_feasible() const;

        // bool is_integer_feasible() const;

        bool has_fractional_solution() const;

    };

    class BBNodeCompare {
    public:
        bool operator()(const std::shared_ptr<BBNode> &n1, const std::shared_ptr<BBNode> &n2) const {
            if(!n1->father_lb) { return true; }
            if(!n2->father_lb) { return false; }
            return (n1->father_lb > n2->father_lb);
        }
    };
}
#endif
