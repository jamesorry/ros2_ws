#!/bin/bash
# gnome-terminal --tab #打開兩個，第一個兩個tab，第二個3個tab
gnome-terminal --tab -t "multi_tb3_simulation_v5_launch" -- zsh -c -i "source ~/.zshrc;killall gzserver ; killall gzclient;ros2 launch my_bringup multi_tb3_simulation_v5_launch.py;exec zsh;"
sleep 20
gnome-terminal --tab -t "ros_recognition_yolo" -- zsh -c -i "source ~/.zshrc;cd ~/ros2_ws/src/yolobot/yolobot_recognition/scripts;python3 ros_recognition_yolo_fix_v10.py;exec zsh;"
sleep 5
gnome-terminal --tab -t "track_action_server" -- zsh -c -i "source ~/.zshrc;cd ~/ros2_ws/src/my_nav2_bringup/my_bringup/src;python3 track_action_server.py -pid True;exec zsh;"
sleep 5
gnome-terminal --tab -t "get_position_node" -- zsh -c "source ~/.zshrc;cd ~/ros2_ws/src/yolobot/yolobot_recognition/scripts;python3 get_position_node_v3.py;exec zsh;"
sleep 5
gnome-terminal --tab -t "plot_pid_curve" -- zsh -c -i "source ~/.zshrc;cd ~/ros2_ws/src/my_nav2_bringup/my_bringup/src;python3 plot_pid_curve.py;exec zsh;"
sleep 5
gnome-terminal --tab -t "rqt_robot_steering" -- zsh -c "source ~/.zshrc;cd;rqt_robot_steering;exec zsh;"