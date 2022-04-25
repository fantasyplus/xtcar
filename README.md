# xtcar
rm -rf build/ devel/

catkin_make

source devel/setup.bash

roslaunch hp_hybrid_astr_node hybrid_astar_test.launch
