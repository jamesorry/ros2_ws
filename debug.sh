clear
colcon build --packages-select box_bot_description box_bot_gazebo
source install/setup.zsh
clear
ros2 launch box_bot_gazebo multi_box_bot_launch.py
