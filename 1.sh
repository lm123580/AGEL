#!/bin/bash

# 打开第一个标签并运行命令
gnome-terminal --tab --title="roscore" -- bash -c "roscore; exec bash"

# 等待3秒
sleep 3s

# 打开第二个标签并运行命令
gnome-terminal --tab --title="rviz" -- bash -c "source ./devel/setup.bash && rviz -d a.rviz; exec bash"

