# cmake最低版本号要求
cmake_minimum_required (VERSION 3.20)
# 设置PROJECT_NAME变量
set(PROJECT_NAME Drone_cover)
# 设置工程名
project (${PROJECT_NAME})       #{PROJECT_NAME}中的PROJECT_NAME就是上面set函数中的PROJECT_NAME
# 设置C++标准为C++11
set(CMAKE_CXX_STANDARD 11)
# 指定外部头文件搜索路径
# include_directories(${PROJECT_SOURCE_DIR}/inc)
include_directories(D:/ALTF/CodePractice/Linux/CoscoModel/include/windows)

#下面这两个也是可以用的，但是不知道为什么在这里用不了
#include_directories(D:/Application/Cplex/concert/include)
#include_directories(D:/Application/Cplex/cplex/include)

# 指定引用的外部库的搜索路径
#如果要使用Debug模式，则用这两行
#  link_directories(D:/Application/Cplex/cplex/lib/x64_windows_vs2017/stat_mdd) # stat_mda为release模式；stat_mdd为debug模式
#  link_directories(D:/Application/Cplex/concert/lib/x64_windows_vs2017/stat_mdd)

#如果要使用Release模式，则用这两行
link_directories(D:/Application/Cplex/cplex/lib/x64_windows_vs2017/stat_mda)
link_directories(D:/Application/Cplex/concert/lib/x64_windows_vs2017/stat_mda)

# 查找当前目录下的所有源文件并存入DIR_SRCS变量
#aux_source_directory(${PROJECT_SOURCE_DIR}/src DIR_SRCS)      #也可以用file(GLOB)函数进行设置
# file(GLOB CPP_FILES_IN_SRC "${PROJECT_SOURCE_DIR}/data/*.cpp")
# set(DIR_SRCS ${CPP_FILES_IN_SRC} ${PROJECT_SOURCE_DIR}/main.cpp)

set(DIR_SRCS
  code/branching/bb_node.h
  code/branching/bb_tree.h
  code/branching/bb_node.cpp
  code/branching/bb_tree.cpp
  code/main.cpp
  
  code/base/arc.h
  code/base/node.h
  code/base/problem.h
  code/base/drone_class.h
  code/base/target.h
  code/base/inspection.h
  code/base/problem.cpp
  code/base/node.cpp
  code/base/drone_class.cpp
  code/base/arc.cpp
  code/base/graph_properties.h
  code/base/graph.h
  code/base/graph.cpp
  code/base/json.hpp

  code/column/column.cpp
  code/column/solution.cpp
  code/column/column.h
  code/column/column_pool.h
  code/column/solution.h

  code/preprocessing/problem_data.h
  code/preprocessing/program_params.h
  code/preprocessing/problem_data.cpp
  code/preprocessing/program_params.cpp
  code/preprocessing/graph_generator.cpp
  code/preprocessing/graph_generator.h

  code/masterproblem/mp_integer_solution.h
  code/masterproblem/mp_linear_solution.h
  code/masterproblem/mp_solver.h
  code/masterproblem/mp_solver.cpp

  code/subproblem/sp_solver.h
  code/subproblem/sp_integer_solution.h
  code/subproblem/sp_solver.cpp
)

# 指定可执行文件存放目录
SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
# 添加一个可编译的目标到工程
add_executable (${PROJECT_NAME} ${DIR_SRCS})
# 链接第三方XX库的库文件
target_link_libraries(${PROJECT_NAME} concert.lib cplex1280.lib ilocplex.lib)  #lib需要用库的名称覆盖掉，若是静态库，则xx.a/xx.lib；若是动态库，则xx。
