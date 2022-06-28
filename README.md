# Hybrid A*


## ompl需要单独编译
见ompl/build_ompl.sh

## 大致编译流程
```
rm -rf build/ devel/
```
```
catkin_make -DCATKIN_WHITELIST_PACKAGES="mpc_msgs"
```
```
catkin_make -DCATKIN_WHITELIST_PACKAGES=""
```
```
source devel/setup.bash
```
//静态场景
```
roslaunch behaviour_state_machine hybrid_astar_test_static.launch
```
//动态场景
```
roslaunch behaviour_state_machine hybrid_astar_test_dynamic.launch
```
