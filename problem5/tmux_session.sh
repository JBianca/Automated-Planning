#!/bin/bash

cd ~/plansys2_ws || exit 1

tmux new-session -d -s problem5
tmux split-window -h -t problem5

# Pane 0.0: build + launch
tmux select-pane -t 0
tmux send-keys "source /opt/ros/humble/setup.bash" C-m
tmux send-keys "rosdep install --from-paths src --ignore-src -r -y" C-m
tmux send-keys "colcon build --symlink-install" C-m
tmux send-keys "source install/setup.bash" C-m
tmux send-keys "ros2 launch problem5 problem5_launch.py" C-m

# # # Pane 0.1: plansys2 terminal
tmux select-pane -t 1
tmux send-keys "sleep 20" C-m
tmux send-keys "cd ~/plansys2_ws/src/problem5/launch" C-m
tmux send-keys "source /opt/ros/humble/setup.bash" C-m
tmux send-keys "source ~/plansys2_ws/install/setup.bash" C-m
tmux send-keys "ros2 run plansys2_terminal plansys2_terminal" C-m

tmux attach -t problem5