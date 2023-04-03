#!/bin/bash
# gnome-terminal --tab #打開兩個，第一個兩個tab，第二個3個tab
# ! 啟動ros2以及整個Gazebo模擬環境
gnome-terminal --tab -t "multi_tb3_simulation_v5_launch" -- zsh -c -i "source ~/.zshrc;killall gzserver ; killall gzclient;ros2 launch my_bringup multi_tb3_simulation_v5_launch.py;exec zsh;"
sleep 20
# ! 啟動YOLOV5
gnome-terminal --tab -t "ros_recognition_yolo" -- zsh -c -i "source ~/.zshrc;cd ~/ros2_ws/src/yolobot/yolobot_recognition/scripts;python3 ros_recognition_yolo_fix_v10.py;exec zsh;"
sleep 5
# ! 啟動跟隨action server
gnome-terminal --tab -t "track_action_server" -- zsh -c -i "source ~/.zshrc;cd ~/ros2_ws/src/my_nav2_bringup/my_bringup/src;python3 track_action_server.py -pid True;exec zsh;"
sleep 5
# ! 監控畫面(顯示機器人座標，顯示辨識目標座標，預測目標移動，控制跟隨)
gnome-terminal --tab -t "get_position_node" -- zsh -c "source ~/.zshrc;cd ~/ros2_ws/src/yolobot/yolobot_recognition/scripts;python3 get_position_node_v3.py;exec zsh;"
sleep 5
# ! 畫pid曲線圖(可以跑出曲線了，但結果還不知道正不正確)
# gnome-terminal --tab -t "plot_pid_curve" -- zsh -c -i "source ~/.zshrc;cd ~/ros2_ws/src/my_nav2_bringup/my_bringup/src;python3 plot_pid_curve.py;exec zsh;"
sleep 5
# ! 打開twist控制器
# gnome-terminal --tab -t "rqt_robot_steering" -- zsh -c "source ~/.zshrc;cd;rqt_robot_steering;exec zsh;"