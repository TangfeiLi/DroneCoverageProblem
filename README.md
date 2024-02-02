# DroneCoverageProblem

## 文件夹  
./data: 计算实验相关数据  
./data/input：实验算例和算例生成的逻辑代码  
./data/result：实验结果  
./codeCG:CG的源代码文件夹
./codeCG/code:CG方法的源代码实现
./codeCG/opt:使用bash脚本执行编译后的.exe文件

## 文件
./codeCG/CMakeLists.txt: 编译源代码的配置文件
./codeCG/code/main.cpp: CG代码的入口文件
./codeCG/launch_jobs.sh: 执行整个项目的.sh文件，在windows平台用git bash命令行使用bash launch_jobs.sh执行该文件
./codeCG/solved_instances.txt: 已经求解的算例文件记录