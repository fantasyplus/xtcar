# Hybrid A*
## 注意：car_simulator对autoware有依赖，不编译成功也没关系

## 大致编译流程
rm -rf build/ devel/

catkin_make

source devel/setup.bash

//静态场景

roslaunch behaviour_state_machine hybrid_astar_test_static.launch

//动态场景

roslaunch behaviour_state_machine hybrid_astar_test_dynamic.launch
