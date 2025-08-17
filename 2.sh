#!/bin/bash
# 打开第一个标签并运行命令
gnome-terminal --tab --title="gazebo" -- bash -c "roslaunch px4 lm_maze_3.launch; exec bash"

# 等待3秒
sleep 10s

gnome-terminal --tab --title="set hz"  -- bash -c "rosrun mavros mavcmd long 511 105 5000 0 0 0 0 0;rosrun mavros mavcmd long 511 31 5000 0 0 0 0 0;rosrun mavros mavcmd long 511 32 10000 0 0 0 0 0; rosrun mavros mavcmd long 511 30 5000 0 0 0 0 0;exec bash"

sleep 1s

gnome-terminal --tab --title="explore system" -- bash -c "source ./devel/setup.bash && roslaunch system_fsm start.launch; exec bash"

