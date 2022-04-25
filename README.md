# Hybrid A*
## 注意：car_simulator对autoware有依赖，不编译成功也没关系

## 大致编译流程
rm -rf build/ devel/

catkin_make

source devel/setup.bash

roslaunch hp_hybrid_astr_node hybrid_astar_test.launch
