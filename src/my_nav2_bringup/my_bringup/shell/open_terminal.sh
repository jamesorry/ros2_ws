#!/bin/bash
# gnome-terminal --tab #打開兩個，第一個兩個tab，第二個3個tab
ros2 daemon stop
sleep 3
ros2 daemon start
sleep 3
killall gzserver ; killall gzclient
sleep 3
ros2 topic list
# ! 啟動ros2以及整個Gazebo模擬環境
gnome-terminal --tab -t "multi_tb3_simulation_v5_launch" -- zsh -c "killall gzserver ; killall gzclient;cd ~/ros2_ws;exec zsh;"
# ! 啟動YOLOV5
gnome-terminal --tab -t "ros_recognition_yolo" -- zsh -c "cd ~/ros2_ws/src/yolobot/yolobot_recognition/scripts;exec zsh;"
# ! 啟動跟隨action server
gnome-terminal --tab -t "track_action_server" -- zsh -c "cd ~/ros2_ws/src/my_nav2_bringup/my_bringup/src;exec zsh;"
# ! 監控畫面(顯示機器人座標，顯示辨識目標座標，預測目標移動，控制跟隨)
gnome-terminal --tab -t "get_position_node" -- zsh -c "cd ~/ros2_ws/src/yolobot/yolobot_recognition/scripts;exec zsh;"
