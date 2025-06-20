# ROS_1110344
巡邏機器人（期末專題）

專案介紹
本專案為期末專題，主題為「巡邏機器人」。本機器人可在室內場景中按照指定路徑自動巡邏，並具備基本閃避障礙能力。

功能
1.巡邏路徑導航
2.雷射感測避障
3.Gazebo 模擬測試
4.ROS 節點控制

執行：
roscore

chmod +x ~/catkin_ws/src/slam_bot_with_fusion360-main/myrobot_description/scripts/patrol_node.py

roslaunch myrobot_description gazebo.launch 

roslaunch myrobot_description patrol.launch
