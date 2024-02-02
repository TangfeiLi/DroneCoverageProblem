

#include <iostream>
#include <string>
#include "branching/bb_tree.h"

int main(int argc, char* argv[]) {
	using namespace drone_cover;

	if(argc != 4) {
        std::cerr << "Usage: ./feeder <params_file> <result_file> <data_file>" << std::endl;
        return -1;
    }

	BBTree tree = BBTree(argv[1], argv[2], argv[3]);
    tree.explore_tree();

	return 0;
}