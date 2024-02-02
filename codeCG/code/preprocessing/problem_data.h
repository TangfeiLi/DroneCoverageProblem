
#ifndef PROBLEM_DATA_H
#define PROBLEM_DATA_H

#include <memory>
#include <string>
#include <utility>
#include <vector>


#include "../base/node.h"
#include "../base/drone_class.h"
#include "../base/target.h"

namespace drone_cover {

    class ProblemData {
    public:
        int num_times;

        int num_drone_classes;
        std::vector<std::shared_ptr<DroneClass>> drone_classes;

        int num_targets;
        std::vector<std::shared_ptr<Target>> targets;

        ProblemData(const std::string &data_file_name);

        void print();
    };
}

#endif
